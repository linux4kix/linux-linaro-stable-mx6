/*
 * zbud.c - Buddy Allocator for Compressed Pages
 *
 * Copyright (C) 2013, Seth Jennings, IBM
 *
 * Concepts based on zcache internal zbud allocator by Dan Magenheimer.
 *
 * zbud is an special purpose allocator for storing compressed pages. It is
 * designed to store up to two compressed pages per physical page.  While this
 * design limits storage density, it has simple and deterministic reclaim
 * properties that make it preferable to a higher density approach when reclaim
 * will be used.
 *
 * zbud works by storing compressed pages, or "zpages", together in pairs in a
 * single memory page called a "zbud page".  The first buddy is "left
 * justifed" at the beginning of the zbud page, and the last buddy is "right
 * justified" at the end of the zbud page.  The benefit is that if either
 * buddy is freed, the freed buddy space, coalesced with whatever slack space
 * that existed between the buddies, results in the largest possible free region
 * within the zbud page.
 *
 * zbud also provides an attractive lower bound on density. The ratio of zpages
 * to zbud pages can not be less than 1.  This ensures that zbud can never "do
 * harm" by using more pages to store zpages than the uncompressed zpages would
 * have used on their own.
 *
 * zbud pages are divided into "chunks".  The size of the chunks is fixed at
 * compile time and determined by NCHUNKS_ORDER below.  Dividing zbud pages
 * into chunks allows organizing unbuddied zbud pages into a manageable number
 * of unbuddied lists according to the number of free chunks available in the
 * zbud page.
 *
 * The zbud API differs from that of conventional allocators in that the
 * allocation function, zbud_alloc(), returns an opaque handle to the user,
 * not a dereferenceable pointer.  The user must map the handle using
 * zbud_map() in order to get a usable pointer by which to access the
 * allocation data and unmap the handle with zbud_unmap() when operations
 * on the allocation data are complete.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/zbud.h>

/*****************
 * Structures
*****************/
/**
 * struct zbud_page - zbud page metadata overlay
 *  <at> page:	typed reference to the underlying struct page
 *  <at> donotuse:	this overlays the page flags and should not be used
 *  <at> first_chunks:	the size of the first buddy in chunks, 0 if free
 *  <at> last_chunks:	the size of the last buddy in chunks, 0 if free
 *  <at> buddy:	links the zbud page into the unbuddied/buddied lists in the pool
 *  <at> lru:	links the zbud page into the lru list in the pool
 *
 * This structure overlays the struct page to store metadata needed for a
 * single storage page in for zbud.  There is a BUILD_BUG_ON in zbud_init()
 * that ensures this structure is not larger that struct page.
 *
 * The PG_reclaim flag of the underlying page is used for indicating
 * that this zbud page is under reclaim (see zbud_reclaim_page())
 */
struct zbud_page {
	union {
		struct page page;
		struct {
			unsigned long donotuse;
			u16 first_chunks;
			u16 last_chunks;
			struct list_head buddy;
			struct list_head lru;
		};
	};
};

/*
 * NCHUNKS_ORDER determines the internal allocation granularity, effectively
 * adjusting internal fragmentation.  It also determines the number of
 * freelists maintained in each pool. NCHUNKS_ORDER of 6 means that the
 * allocation granularity will be in chunks of size PAGE_SIZE/64, and there
 * will be 64 freelists per pool.
 */
#define NCHUNKS_ORDER	6

#define CHUNK_SHIFT	(PAGE_SHIFT - NCHUNKS_ORDER)
#define CHUNK_SIZE	(1 << CHUNK_SHIFT)
#define NCHUNKS		(PAGE_SIZE >> CHUNK_SHIFT)

/**
 * struct zbud_pool - stores metadata for each zbud pool
 *  <at> lock:	protects all pool lists and first|last_chunk fields of any
 *		zbud page in the pool
 *  <at> unbuddied:	array of lists tracking zbud pages that only contain one buddy;
 *		the lists each zbud page is added to depends on the size of
 *		its free region.
 *  <at> buddied:	list tracking the zbud pages that contain two buddies;
 *		these zbud pages are full
 *  <at> pages_nr:	number of zbud pages in the pool.
 *  <at> ops:	pointer to a structure of user defined operations specified at
 *		pool creation time.
 *
 * This structure is allocated at pool creation time and maintains metadata
 * pertaining to a particular zbud pool.
 */
struct zbud_pool {
	spinlock_t lock;
	struct list_head unbuddied[NCHUNKS];
	struct list_head buddied;
	struct list_head lru;
	atomic_t pages_nr;
	struct zbud_ops *ops;
};

/*****************
 * Helpers
*****************/
/* Just to make the code easier to read */
enum buddy {
	FIRST,
	LAST
};

/* Converts an allocation size in bytes to size in zbud chunks */
static inline int size_to_chunks(int size)
{
	return (size + CHUNK_SIZE - 1) >> CHUNK_SHIFT;
}

#define for_each_unbuddied_list(_iter, _begin) \
	for ((_iter) = (_begin); (_iter) < NCHUNKS; (_iter)++)

/* Initializes a zbud page from a newly allocated page */
static inline struct zbud_page *init_zbud_page(struct page *page)
{
	struct zbud_page *zbpage = (struct zbud_page *)page;
	zbpage->first_chunks = 0;
	zbpage->last_chunks = 0;
	INIT_LIST_HEAD(&zbpage->buddy);
	INIT_LIST_HEAD(&zbpage->lru);
	return zbpage;
}

/* Resets a zbud page so that it can be properly freed  */
static inline struct page *reset_zbud_page(struct zbud_page *zbpage)
{
	struct page *page = &zbpage->page;
	set_page_private(page, 0);
	page->mapping = NULL;
	page->index = 0;
	page_mapcount_reset(page);
	init_page_count(page);
	INIT_LIST_HEAD(&page->lru);
	return page;
}

/*
 * Encodes the handle of a particular buddy within a zbud page
 * Pool lock should be held as this function accesses first|last_chunks
 */
static inline unsigned long encode_handle(struct zbud_page *zbpage,
					enum buddy bud)
{
	unsigned long handle;

	/*
	 * For now, the encoded handle is actually just the pointer to the data
	 * but this might not always be the case.  A little information hiding.
	 */
	handle = (unsigned long)page_address(&zbpage->page);
	if (bud == FIRST)
		return handle;
	handle += PAGE_SIZE - (zbpage->last_chunks  << CHUNK_SHIFT);
	return handle;
}

/* Returns the zbud page where a given handle is stored */
static inline struct zbud_page *handle_to_zbud_page(unsigned long handle)
{
	return (struct zbud_page *)(virt_to_page(handle));
}

/* Returns the number of free chunks in a zbud page */
static inline int num_free_chunks(struct zbud_page *zbpage)
{
	/*
	 * Rather than branch for different situations, just use the fact that
	 * free buddies have a length of zero to simplify everything.
	 */
	return NCHUNKS - zbpage->first_chunks - zbpage->last_chunks;
}

/*****************
 * API Functions
*****************/
/**
 * zbud_create_pool() - create a new zbud pool
 *  <at> gfp:	gfp flags when allocating the zbud pool structure
 *  <at> ops:	user-defined operations for the zbud pool
 *
 * Return: pointer to the new zbud pool or NULL if the metadata allocation
 * failed.
 */
struct zbud_pool *zbud_create_pool(gfp_t gfp, struct zbud_ops *ops)
{
	struct zbud_pool *pool;
	int i;

	pool = kmalloc(sizeof(struct zbud_pool), gfp);
	if (!pool)
		return NULL;
	spin_lock_init(&pool->lock);
	for_each_unbuddied_list(i, 0)
		INIT_LIST_HEAD(&pool->unbuddied[i]);
	INIT_LIST_HEAD(&pool->buddied);
	INIT_LIST_HEAD(&pool->lru);
	atomic_set(&pool->pages_nr, 0);
	pool->ops = ops;
	return pool;
}
EXPORT_SYMBOL_GPL(zbud_create_pool);

/**
 * zbud_destroy_pool() - destroys an existing zbud pool
 *  <at> pool:	the zbud pool to be destroyed
 */
void zbud_destroy_pool(struct zbud_pool *pool)
{
	kfree(pool);
}
EXPORT_SYMBOL_GPL(zbud_destroy_pool);

/**
 * zbud_alloc() - allocates a region of a given size
 *  <at> pool:	zbud pool from which to allocate
 *  <at> size:	size in bytes of the desired allocation
 *  <at> gfp:	gfp flags used if the pool needs to grow
 *  <at> handle:	handle of the new allocation
 *
 * This function will attempt to find a free region in the pool large
 * enough to satisfy the allocation request.  First, it tries to use
 * free space in the most recently used zbud page, at the beginning of
 * the pool LRU list.  If that zbud page is full or doesn't have the
 * required free space, a best fit search of the unbuddied lists is
 * performed. If no suitable free region is found, then a new page
 * is allocated and added to the pool to satisfy the request.
 *
 * gfp should not set __GFP_HIGHMEM as highmem pages cannot be used
 * as zbud pool pages.
 *
 * Return: 0 if success and handle is set, otherwise -EINVAL is the size or
 * gfp arguments are invalid or -ENOMEM if the pool was unable to allocate
 * a new page.
 */
int zbud_alloc(struct zbud_pool *pool, int size, gfp_t gfp,
			unsigned long *handle)
{
	int chunks, i, freechunks;
	struct zbud_page *zbpage = NULL;
	enum buddy bud;
	struct page *page;

	if (size <= 0 || size > PAGE_SIZE || gfp & __GFP_HIGHMEM)
		return -EINVAL;
	chunks = size_to_chunks(size);
	spin_lock(&pool->lock);

	/*
	 * First, try to use the zbpage we last used (at the head of the
	 * LRU) to increase LRU locality of the buddies. This is first fit.
	 */
	if (!list_empty(&pool->lru)) {
		zbpage = list_first_entry(&pool->lru, struct zbud_page, lru);
		if (num_free_chunks(zbpage) >= chunks) {
			if (zbpage->first_chunks == 0) {
				list_del(&zbpage->buddy);
				bud = FIRST;
				goto found;
			}
			if (zbpage->last_chunks == 0) {
				list_del(&zbpage->buddy);
				bud = LAST;
				goto found;
			}
		}
	}

	/* Second, try to find an unbuddied zbpage. This is best fit. */
	zbpage = NULL;
	for_each_unbuddied_list(i, chunks) {
		if (!list_empty(&pool->unbuddied[i])) {
			zbpage = list_first_entry(&pool->unbuddied[i],
					struct zbud_page, buddy);
			list_del(&zbpage->buddy);
			if (zbpage->first_chunks == 0)
				bud = FIRST;
			else
				bud = LAST;
			goto found;
		}
	}

	/* Lastly, couldn't find unbuddied zbpage, create new one */
	spin_unlock(&pool->lock);
	page = alloc_page(gfp);
	if (!page)
		return -ENOMEM;
	spin_lock(&pool->lock);
	atomic_inc(&pool->pages_nr);
	zbpage = init_zbud_page(page);
	bud = FIRST;

found:
	if (bud == FIRST)
		zbpage->first_chunks = chunks;
	else
		zbpage->last_chunks = chunks;

	if (zbpage->first_chunks == 0 || zbpage->last_chunks == 0) {
		/* Add to unbuddied list */
		freechunks = num_free_chunks(zbpage);
		list_add(&zbpage->buddy, &pool->unbuddied[freechunks]);
	} else {
		/* Add to buddied list */
		list_add(&zbpage->buddy, &pool->buddied);
	}

	/* Add/move zbpage to beginning of LRU */
	if (!list_empty(&zbpage->lru))
		list_del(&zbpage->lru);
	list_add(&zbpage->lru, &pool->lru);

	*handle = encode_handle(zbpage, bud);
	spin_unlock(&pool->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(zbud_alloc);

/**
 * zbud_free() - frees the allocation associated with the given handle
 *  <at> pool:	pool in which the allocation resided
 *  <at> handle:	handle associated with the allocation returned by zbud_alloc()
 *
 * In the case that the zbud page in which the allocation resides is under
 * reclaim, as indicated by the PG_reclaim flag being set, this function
 * only sets the first|last_chunks to 0.  The page is actually freed
 * once both buddies are evicted (see zbud_reclaim_page() below).
 */
void zbud_free(struct zbud_pool *pool, unsigned long handle)
{
	struct zbud_page *zbpage;
	int freechunks;

	spin_lock(&pool->lock);
	zbpage = handle_to_zbud_page(handle);

	/* If first buddy, handle will be page aligned */
	if (handle & ~PAGE_MASK)
		zbpage->last_chunks = 0;
	else
		zbpage->first_chunks = 0;

	if (PageReclaim(&zbpage->page)) {
		/* zbpage is under reclaim, reclaim will free */
		spin_unlock(&pool->lock);
		return;
	}

	/* Remove from existing buddy list */
	list_del(&zbpage->buddy);

	if (zbpage->first_chunks == 0 && zbpage->last_chunks == 0) {
		/* zbpage is empty, free */
		list_del(&zbpage->lru);
		__free_page(reset_zbud_page(zbpage));
		atomic_dec(&pool->pages_nr);
	} else {
		/* Add to unbuddied list */
		freechunks = num_free_chunks(zbpage);
		list_add(&zbpage->buddy, &pool->unbuddied[freechunks]);
	}

	spin_unlock(&pool->lock);
}
EXPORT_SYMBOL_GPL(zbud_free);

#define list_tail_entry(ptr, type, member) \
	list_entry((ptr)->prev, type, member)

/**
 * zbud_reclaim_page() - evicts allocations from a pool page and frees it
 *  <at> pool:	pool from which a page will attempt to be evicted
 *  <at> retires:	number of pages on the LRU list for which eviction will
 *		be attempted before failing
 *
 * zbud reclaim is different from normal system reclaim in that the reclaim is
 * done from the bottom, up.  This is because only the bottom layer, zbud, has
 * information on how the allocations are organized within each zbud page. This
 * has the potential to create interesting locking situations between zbud and
 * the user, however.
 *
 * To avoid these, this is how zbud_reclaim_page() should be called:

 * The user detects a page should be reclaimed and calls zbud_reclaim_page().
 * zbud_reclaim_page() will remove a zbud page from the pool LRU list and call
 * the user-defined eviction handler with the pool and handle as arguments.
 *
 * If the handle can not be evicted, the eviction handler should return
 * non-zero. zbud_reclaim_page() will add the zbud page back to the
 * appropriate list and try the next zbud page on the LRU up to
 * a user defined number of retries.
 *
 * If the handle is successfully evicted, the eviction handler should
 * return 0 _and_ should have called zbud_free() on the handle. zbud_free()
 * contains logic to delay freeing the page if the page is under reclaim,
 * as indicated by the setting of the PG_reclaim flag on the underlying page.
 *
 * If all buddies in the zbud page are successfully evicted, then the
 * zbud page can be freed.
 *
 * Returns: 0 if page is successfully freed, otherwise -EINVAL if there are
 * no pages to evict or an eviction handler is not registered, -EAGAIN if
 * the retry limit was hit.
 */
int zbud_reclaim_page(struct zbud_pool *pool, unsigned int retries)
{
	int i, ret, freechunks;
	struct zbud_page *zbpage;
	unsigned long first_handle = 0, last_handle = 0;

	spin_lock(&pool->lock);
	if (!pool->ops || !pool->ops->evict || list_empty(&pool->lru) ||
			retries == 0) {
		spin_unlock(&pool->lock);
		return -EINVAL;
	}
	for (i = 0; i < retries; i++) {
		zbpage = list_tail_entry(&pool->lru, struct zbud_page, lru);
		list_del(&zbpage->lru);
		list_del(&zbpage->buddy);
		/* Protect zbpage against free */
		SetPageReclaim(&zbpage->page);
		/*
		 * We need encode the handles before unlocking, since we can
		 * race with free that will set (first|last)_chunks to 0
		 */
		first_handle = 0;
		last_handle = 0;
		if (zbpage->first_chunks)
			first_handle = encode_handle(zbpage, FIRST);
		if (zbpage->last_chunks)
			last_handle = encode_handle(zbpage, LAST);
		spin_unlock(&pool->lock);

		/* Issue the eviction callback(s) */
		if (first_handle) {
			ret = pool->ops->evict(pool, first_handle);
			if (ret)
				goto next;
		}
		if (last_handle) {
			ret = pool->ops->evict(pool, last_handle);
			if (ret)
				goto next;
		}
next:
		spin_lock(&pool->lock);
		ClearPageReclaim(&zbpage->page);
		if (zbpage->first_chunks == 0 && zbpage->last_chunks == 0) {
			/*
			 * Both buddies are now free, free the zbpage and
			 * return success.
			 */
			__free_page(reset_zbud_page(zbpage));
			atomic_dec(&pool->pages_nr);
			spin_unlock(&pool->lock);
			return 0;
		} else if (zbpage->first_chunks == 0 ||
				zbpage->last_chunks == 0) {
			/* add to unbuddied list */
			freechunks = num_free_chunks(zbpage);
			list_add(&zbpage->buddy, &pool->unbuddied[freechunks]);
		} else {
			/* add to buddied list */
			list_add(&zbpage->buddy, &pool->buddied);
		}

		/* add to beginning of LRU */
		list_add(&zbpage->lru, &pool->lru);
	}
	spin_unlock(&pool->lock);
	return -EAGAIN;
}
EXPORT_SYMBOL_GPL(zbud_reclaim_page);

/**
 * zbud_map() - maps the allocation associated with the given handle
 *  <at> pool:	pool in which the allocation resides
 *  <at> handle:	handle associated with the allocation to be mapped
 *
 * While trivial for zbud, the mapping functions for others allocators
 * implementing this allocation API could have more complex information encoded
 * in the handle and could create temporary mappings to make the data
 * accessible to the user.
 *
 * Returns: a pointer to the mapped allocation
 */
void *zbud_map(struct zbud_pool *pool, unsigned long handle)
{
	return (void *)(handle);
}
EXPORT_SYMBOL_GPL(zbud_map);

/**
 * zbud_unmap() - maps the allocation associated with the given handle
 *  <at> pool:	pool in which the allocation resides
 *  <at> handle:	handle associated with the allocation to be unmapped
 */
void zbud_unmap(struct zbud_pool *pool, unsigned long handle)
{
}
EXPORT_SYMBOL_GPL(zbud_unmap);

/**
 * zbud_get_pool_size() - gets the zbud pool size in pages
 *  <at> pool:	pool whose size is being queried
 *
 * Returns: size in pages of the given pool
 */
int zbud_get_pool_size(struct zbud_pool *pool)
{
	return atomic_read(&pool->pages_nr);
}
EXPORT_SYMBOL_GPL(zbud_get_pool_size);

static int __init init_zbud(void)
{
	/* Make sure we aren't overflowing the underlying struct page */
	BUILD_BUG_ON(sizeof(struct zbud_page) != sizeof(struct page));
	/* Make sure we can represent any chunk offset with a u16 */
	BUILD_BUG_ON(sizeof(u16) * BITS_PER_BYTE < PAGE_SHIFT - CHUNK_SHIFT);
	pr_info("loaded\n");
	return 0;
}

static void __exit exit_zbud(void)
{
	pr_info("unloaded\n");
}

module_init(init_zbud);
module_exit(exit_zbud);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Seth Jennings <sjenning <at> linux.vnet.ibm.com>");
MODULE_DESCRIPTION("Buddy Allocator for Compressed Pages");
