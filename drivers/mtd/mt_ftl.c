#include "mt_ftl.h"
#include "ubi/ubi.h"
#include <linux/crypto.h>
#ifdef MT_FTL_PROFILE
#include <linux/time.h>

unsigned long profile_time[MT_FTL_PROFILE_TOTAL_PROFILE_NUM];
unsigned long start_time = 0, end_time = 0;
unsigned long start_time_all = 0, end_time_all = 0;

unsigned long getnstimenow(void)
{
	struct timespec tv;

	getnstimeofday(&tv);
	return tv.tv_sec * 1000000000 + tv.tv_nsec;
}

#define MT_FTL_PROFILE_START(x)		(start_time = getnstimenow())

#define MT_FTL_PROFILE_START_ALL(x)	(start_time_all = getnstimenow())

#define MT_FTL_PROFILE_END(x)	\
	do { \
		end_time = getnstimenow(); \
		if (end_time >= start_time) \
			profile_time[x] += (end_time - start_time) / 1000; \
		else { \
			mt_ftl_err("end_time = %lu, start_time = %lu", end_time, start_time);\
			profile_time[x] += (end_time + 0xFFFFFFFF - start_time) / 1000; \
		} \
	} while (0)
#define MT_FTL_PROFILE_END_ALL(x)	\
	do { \
		end_time_all = getnstimenow(); \
		if (end_time_all >= start_time_all) \
			profile_time[x] += (end_time_all - start_time_all) / 1000; \
		else { \
			mt_ftl_err("end_time = %lu, start_time = %lu", end_time_all, start_time_all); \
			profile_time[x] += (end_time_all + 0xFFFFFFFF - start_time_all) / 1000; \
		} \
	} while (0)

#else
#define MT_FTL_PROFILE_START(x)
#define MT_FTL_PROFILE_START_ALL(x)
#define MT_FTL_PROFILE_END(x)
#define MT_FTL_PROFILE_END_ALL(x)
#endif
/* recorded the pmt block for recovery */
static int recorded_pmt_blk = -1;
/* recorded the last replay for pmt recover*/
static int last_replay_flag = REPLAY_EMPTY;

#ifdef MT_FTL_SUPPORT_COMPR
static int mt_ftl_compress(struct mt_ftl_blk *dev, const void *in_buf, int in_len, int *out_len)
{
	int ret = MT_FTL_SUCCESS;
	struct mt_ftl_param *param = dev->param;

	ubi_assert(in_len <= FS_PAGE_SIZE);
	memset(param->cmpr_page_buffer, 0xFF, dev->min_io_size * sizeof(unsigned char));
	ret = crypto_comp_compress(param->cc, in_buf, in_len, param->cmpr_page_buffer, out_len);
	if (ret) {
		mt_ftl_err("ret=%d, out_len=%d, in_len=0x%x", ret, *out_len, in_len);
		mt_ftl_err("cc=0x%lx, buffer=0x%lx", (unsigned long int)param->cc, (unsigned long int)in_buf);
		mt_ftl_err("cmpr_page_buffer=0x%lx", (unsigned long int)param->cmpr_page_buffer);
		return MT_FTL_FAIL;
	}
	if (*out_len >= FS_PAGE_SIZE) {
		/* mt_ftl_err(dev, "compress out_len(%d) large than in_len(%d)FS_PAGE_SIZE", *out_len, in_len); */
		memcpy(param->cmpr_page_buffer, in_buf, in_len);
		*out_len = in_len;
	}
	return ret;
}

static int mt_ftl_decompress(struct mt_ftl_blk *dev, const void *in_buf, int in_len, int *out_len)
{
	int ret = MT_FTL_SUCCESS;
	struct mt_ftl_param *param = dev->param;

	ubi_assert(in_len <= FS_PAGE_SIZE);
	memset(param->cmpr_page_buffer, 0xFF, dev->min_io_size * sizeof(unsigned char));
	if (in_len == FS_PAGE_SIZE) {
		/* mt_ftl_err(dev, "decompress out_len(%d) equal (FS_PAGE_SIZE)", in_len); */
		memcpy(param->cmpr_page_buffer, in_buf, in_len);
		*out_len = in_len;
	} else {
		ret = crypto_comp_decompress(param->cc, in_buf, in_len, param->cmpr_page_buffer, out_len);
		if (ret) {
			mt_ftl_err("ret=%d, out_len=%d, in_len=0x%x", ret, *out_len, in_len);
			mt_ftl_err("cc=0x%lx, buffer=0x%lx", (unsigned long int)param->cc, (unsigned long int)in_buf);
			mt_ftl_err("cmpr_page_buffer=0x%lx", (unsigned long int)param->cmpr_page_buffer);
			return MT_FTL_FAIL;
		}
	}
	return ret;
}
#endif

static int mt_ftl_leb_map(struct mt_ftl_blk *dev, int lnum)
{
	int ret = MT_FTL_SUCCESS;
	struct ubi_volume_desc *desc = dev->desc;

	ret = ubi_is_mapped(desc, lnum);
	if (ret == 0) {
		mt_ftl_err("leb %d is unmapped", lnum);
		ubi_leb_map(desc, lnum);
	}

	return ret;
}

static void mt_ftl_leb_remap(struct mt_ftl_blk *dev, int lnum)
{
	struct ubi_volume_desc *desc = dev->desc;

	ubi_leb_unmap(desc, lnum);
	mt_ftl_err("leb%d has been unmapped, eba_tbl[%d]=%d", lnum, lnum, desc->vol->eba_tbl[lnum]);
	ubi_leb_map(desc, lnum);
	mt_ftl_err("leb%d has been mapped, eba_tbl[%d]=%d", lnum, lnum, desc->vol->eba_tbl[lnum]);
}

static int mt_ftl_leb_read(struct mt_ftl_blk *dev, int lnum, void *buf, int offset, int len)
{
	int ret = MT_FTL_SUCCESS;
	struct ubi_volume_desc *desc = dev->desc;

	ret = ubi_leb_read(desc, lnum, (char *)buf, offset, len, 0);
	if (ret == UBI_IO_BITFLIPS) {
		ret = MT_FTL_SUCCESS;
	} else if (ret) {
		mt_ftl_err("Read failed:leb=%u,offset=%d ret=%d", lnum, offset, ret);
		ret = MT_FTL_FAIL;
		ubi_assert(false);
	}
	cond_resched();
	return ret;
}


static int mt_ftl_leb_write(struct mt_ftl_blk *dev, int lnum, const void *buf, int offset, int len)
{
	int ret = MT_FTL_SUCCESS;
	struct ubi_volume_desc *desc = dev->desc;

	cond_resched();
	ret = ubi_leb_write(desc, lnum, buf, offset, len);
	if (ret) {
		mt_ftl_err("Write failed:leb=%u,offset=%d ret=%d", lnum, offset, ret);
		ret = MT_FTL_FAIL;
		ubi_assert(false);
	}
	cond_resched();
	return ret;
}

static int mt_ftl_leb_recover(struct mt_ftl_blk *dev, int lnum, int size, int check)
{
	int ret = MT_FTL_FAIL, offset = size;
	unsigned char *tmpbuf = NULL;
	struct mt_ftl_param *param = dev->param;

	if ((size == 0) || (size >= dev->leb_size)) {
		mt_ftl_err("size(%d) leb_size(%d) no need recover", size, dev->leb_size);
		return MT_FTL_SUCCESS;
	}
	if (check) {
		ret = mt_ftl_leb_read(dev, lnum, param->replay_page_buffer, offset, dev->min_io_size);
		if (ret == MT_FTL_SUCCESS) {
			offset += dev->min_io_size;
			if (offset >= dev->leb_size) {
				mt_ftl_err("offset(%d) over leb_size(%d)", offset, dev->leb_size);
				return ret;
			}
			mt_ftl_err("lnum(%d) read offset(%d) OK, read next(%d) for check", lnum, size, offset);
			ret = mt_ftl_leb_read(dev, lnum, param->replay_page_buffer, offset, dev->min_io_size);
		}
	}
	if (ret == MT_FTL_FAIL) {
		mt_ftl_err("recover leb(%d) size(%d)", lnum, size);
		tmpbuf = vmalloc(dev->leb_size);
		if (!tmpbuf) {
			ubi_assert(false);
			ret = MT_FTL_FAIL;
			goto out;
		}
		ret = mt_ftl_leb_read(dev, lnum, tmpbuf, 0, size);
		if (ret)
			goto out;
		mt_ftl_leb_remap(dev, lnum);
		ret = mt_ftl_leb_write(dev, lnum, tmpbuf, 0, size);
		if (ret)
			goto out;
		ret = 1; /* recover ok */
	}
out:
	if (tmpbuf) {
		vfree(tmpbuf);
		tmpbuf = NULL;
	}
	return ret;
}

static int mt_ftl_check_empty(const u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (buf[i] != 0xFF)
			return 0;
	}
	return 1;
}

/* The process for PMT should traverse PMT indicator
 * if block num of items in PMT indicator is equal to u4SrcInvalidLeb, copy the
 * corresponding page to new block, and update corresponding PMT indicator */
static int mt_ftl_gc_pmt(struct mt_ftl_blk *dev, int sleb, int dleb, int *offset_dst, bool replay)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int offset_src = 0, dirty = 0;
	int page = 0, cache_num = 0;
	int pmt_len = dev->min_io_size << 1;
	struct mt_ftl_param *param = dev->param;
	unsigned char *gc_pmt_buffer = NULL;

	gc_pmt_buffer = vmalloc(pmt_len);
	if (!gc_pmt_buffer) {
		ret = MT_FTL_FAIL;
		ubi_assert(false);
		goto out;
	}
	if (!replay)
		ret = mt_ftl_leb_map(dev, dleb);
	if (last_replay_flag == REPLAY_FREE_BLK) {
		ret = mt_ftl_leb_map(dev, sleb);
		if (ret > 0) {
			ret = mt_ftl_leb_read(dev, sleb, gc_pmt_buffer, offset_src, pmt_len);
			if (ret)
				goto out;
			ret = mt_ftl_check_empty(gc_pmt_buffer, pmt_len);
			if (ret == 0) {
				mt_ftl_err("[Bean]src_leb(%d) not empty, need to do", sleb);
				mt_ftl_leb_remap(dev, dleb);
				MT_FTL_PARAM_LOCK(dev);
				last_replay_flag = REPLAY_CLEAN_BLK;
				MT_FTL_PARAM_UNLOCK(dev);
				replay = false;
			} else {
				MT_FTL_PARAM_LOCK(dev);
				last_replay_flag = REPLAY_END;
				MT_FTL_PARAM_UNLOCK(dev);
			}
		} else {
			MT_FTL_PARAM_LOCK(dev);
			last_replay_flag = REPLAY_END;
			MT_FTL_PARAM_UNLOCK(dev);
		}
	}
	/* use tmp_page_buffer replace of gc_page_buffer, because gc data would update PMT & gc pmt */
	for (i = 0; i < PMT_TOTAL_CLUSTER_NUM; i++) {
		if (PMT_INDICATOR_GET_BLOCK(param->u4PMTIndicator[i]) == sleb) {
			offset_src = PMT_INDICATOR_GET_PAGE(param->u4PMTIndicator[i]) * dev->min_io_size;
			if (!replay) {
				/* Copy PMT & META PMT */
				mt_ftl_err("Copy PMT&META PMT %d %d %d %d", sleb, offset_src, dleb, *offset_dst);
				ret = mt_ftl_leb_read(dev, sleb, gc_pmt_buffer, offset_src, pmt_len);
				if (ret)
					goto out;
				ret = mt_ftl_leb_write(dev, dleb, gc_pmt_buffer, *offset_dst, pmt_len);
				if (ret)
					goto out;
			}
			dirty = PMT_INDICATOR_IS_DIRTY(param->u4PMTIndicator[i]);
			cache_num = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[i]);
			page = *offset_dst / dev->min_io_size;
			PMT_INDICATOR_SET_BLOCKPAGE(param->u4PMTIndicator[i], dleb, page, dirty, cache_num);
			mt_ftl_err("u4PMTIndicator[%d] = 0x%x, dirty = %d, cache_num = %d, page = %d %d %d\n",
				i, param->u4PMTIndicator[i], dirty, cache_num, page, ret, pmt_len);	/* Temporary */
			*offset_dst += pmt_len;
		}
	}
out:
	if (gc_pmt_buffer) {
		vfree(gc_pmt_buffer);
		gc_pmt_buffer = NULL;
	}
	return ret;
}

static int mt_ftl_check_header(struct mt_ftl_blk *dev, struct mt_ftl_data_header *header, int num)
{
	int i;
	sector_t sec = NAND_DEFAULT_SECTOR_VALUE;

	for (i = 0; i < num; i++)
		sec &= header[i].sector;

	if (sec == NAND_DEFAULT_SECTOR_VALUE) {
		mt_ftl_err("all sectors are 0xFFFFFFFF");
		for (i = 0; i < num; i++)
			mt_ftl_err("header_buffer[%d].sector = 0x%lx", i, (unsigned long int)header[i].sector);
		return 0;
	}
	return 1;
}

static int mt_ftl_get_pmt(struct mt_ftl_blk *dev, sector_t sector, u32 *pmt, u32 *meta_pmt)
{
	int ret = MT_FTL_SUCCESS;
	u32 cluster = 0, sec_offset = 0;
	int cache_num = 0;
	int pm_per_page = dev->pm_per_io;
	int page_size = dev->min_io_size;
	int pmt_block = 0, pmt_page = 0;
	struct mt_ftl_param *param = dev->param;

	/* Calculate clusters and sec_offsets */
	cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / pm_per_page;
	sec_offset = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (pm_per_page - 1);
	ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
	/* Download PMT to read PMT cache */
	/* Don't use mt_ftl_updatePMT, that will cause PMT indicator mixed in replay */
	if (PMT_INDICATOR_IS_INCACHE(param->u4PMTIndicator[cluster])) {
		cache_num = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[cluster]);
		ubi_assert(cache_num < PMT_CACHE_NUM);
		ubi_assert(sec_offset < pm_per_page);
		*pmt = param->u4PMTCache[cache_num * pm_per_page + sec_offset];
		*meta_pmt = param->u4MetaPMTCache[cache_num * pm_per_page + sec_offset];
	} else if (cluster == param->i4CurrentReadPMTClusterInCache) {
		/* mt_ftl_err(dev, "cluster == i4CurrentReadPMTClusterInCache (%d)",
		   param->i4CurrentReadPMTClusterInCache); */
		ubi_assert(sec_offset < pm_per_page);
		*pmt = param->u4ReadPMTCache[sec_offset];
		*meta_pmt = param->u4ReadMetaPMTCache[sec_offset];
	} else {
		pmt_block = PMT_INDICATOR_GET_BLOCK(param->u4PMTIndicator[cluster]);
		pmt_page = PMT_INDICATOR_GET_PAGE(param->u4PMTIndicator[cluster]);

		if (unlikely(pmt_block == 0)) {
			mt_ftl_debug("pmt_block == 0");
			memset(param->u4ReadPMTCache, 0xFF, pm_per_page * sizeof(unsigned int));
			memset(param->u4ReadMetaPMTCache, 0xFF, pm_per_page * sizeof(unsigned int));
			param->i4CurrentReadPMTClusterInCache = 0xFFFFFFFF;
		} else {
			mt_ftl_debug("Get PMT of cluster (%d)", cluster);
			ret = mt_ftl_leb_read(dev, pmt_block, param->u4ReadPMTCache,
				pmt_page * page_size, page_size);
			if (ret)
				goto out;
			ret = mt_ftl_leb_read(dev, pmt_block, param->u4ReadMetaPMTCache,
					(pmt_page + 1) * page_size, page_size);
			if (ret)
				goto out;
			param->i4CurrentReadPMTClusterInCache = cluster;
		}
		ubi_assert(sec_offset <= pm_per_page);
		*pmt = param->u4ReadPMTCache[sec_offset];
		*meta_pmt = param->u4ReadMetaPMTCache[sec_offset];
	}

	if (*pmt == MT_INVALID_BLOCKPAGE) {
		mt_ftl_err("PMT of sector(0x%lx) is invalid", (unsigned long int)sector);
		ret = MT_FTL_FAIL;
		goto out;
	}
out:
	return ret;
}

/*
static void dump_valid_data(struct mt_ftl_blk *dev, struct mt_ftl_valid_data *valid_data)
{
	mt_ftl_err(dev, "GC Dump valid_data info:");
	mt_ftl_err(dev, "\t info.src_leb: %d", valid_data->info.src_leb);
	mt_ftl_err(dev, "\t info.page_src: %d", valid_data->info.page_src);
	mt_ftl_err(dev, "\t info.dst_leb: %d", valid_data->info.dst_leb);
	mt_ftl_err(dev, "\t info.page_dst: %d", valid_data->info.page_dst);
	mt_ftl_err(dev, "\t info.offset_src: %d", valid_data->info.offset_src);
	mt_ftl_err(dev, "\t info.offset_dst: %d", valid_data->info.offset_dst);
	mt_ftl_err(dev, "\t valid_data_num: %d", valid_data->valid_data_num);
	mt_ftl_err(dev, "\t valid_data_offset: %d", valid_data->valid_data_offset);
	mt_ftl_err(dev, "\t valid_buffer: 0x%08x", valid_data->valid_buffer[0]);
}
*/

static int mt_ftl_gc_check_data(struct mt_ftl_blk *dev, struct mt_ftl_data_header *header_buffer,
	struct mt_ftl_valid_data *valid_data, int data_num, int *last_data_len, bool replay)
{
	int i = 0, ret = MT_FTL_SUCCESS;
	int last_data_len_src = 0;
	u32 pmt = 0, meta_pmt = 0;
	sector_t sector = 0;
	u32 total_consumed_size = 0;
	u32 offset_in_page = 0, data_len = 0;
	int page_size = dev->min_io_size;
	int pm_per_page = dev->pm_per_io;
	struct mt_ftl_data_header valid_header;
	u32 cluster = 0, sec_offset = 0, check_num = 0;
	struct mt_ftl_param *param = dev->param;
	char *last_data_buffer = NULL;

	last_data_buffer = vmalloc(FS_PAGE_SIZE);
	if (!last_data_buffer) {
		ret = MT_FTL_FAIL;
		ubi_assert(false);
		goto out;
	}

	for (i = 0; i < data_num; i++) {
		/* Get sector in the page */
		sector = header_buffer[data_num - i - 1].sector;
		if ((sector & NAND_DEFAULT_SECTOR_VALUE) == NAND_DEFAULT_SECTOR_VALUE)
			continue;
		ret = mt_ftl_get_pmt(dev, sector, &pmt, &meta_pmt);
		if (ret)
			goto out;
		cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / pm_per_page;
		sec_offset =
		    ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (pm_per_page - 1);
		ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
		if ((!replay) &&
			(valid_data->info.src_leb == PMT_GET_BLOCK(pmt)) &&
			(valid_data->info.page_src == PMT_GET_PAGE(pmt)) &&
			(i == PMT_GET_PART(meta_pmt))) {
			offset_in_page = (header_buffer[data_num - i - 1].offset_len >> 16) & 0xFFFF;
			data_len = header_buffer[data_num - i - 1].offset_len & 0xFFFF;
			ubi_assert((data_len <= FS_PAGE_SIZE) && (offset_in_page < page_size));
			last_data_len_src = 0;
			total_consumed_size = valid_data->valid_data_offset + data_len +
				(valid_data->valid_data_num + 1) * sizeof(struct mt_ftl_data_header) + 4;
			if ((total_consumed_size > page_size) ||
					valid_data->valid_data_num >= MTKFTL_MAX_DATA_NUM_PER_PAGE) {
				/* dump_valid_data(dev, valid_data); */
				if ((offset_in_page + data_len +
					data_num * sizeof(struct mt_ftl_data_header) + 4) > page_size) {
					last_data_len_src = page_size -
						data_num * sizeof(struct mt_ftl_data_header) - offset_in_page - 4;
					memcpy(last_data_buffer,
						(char *)param->gc_page_buffer + offset_in_page, last_data_len_src);
					ret = mt_ftl_leb_read(dev, valid_data->info.src_leb,
						param->tmp_page_buffer, valid_data->info.offset_src + page_size,
						page_size);
					if (ret)
						goto out;
					memcpy((char *)last_data_buffer + last_data_len_src,
						(char *)param->tmp_page_buffer, (data_len - last_data_len_src));
					check_num =
						PAGE_GET_DATA_NUM(param->tmp_page_buffer[(page_size >> 2) - 1]);
					if (check_num == 0x7FFFFFFF) {
						valid_data->info.page_src += 1;
						valid_data->info.offset_src += page_size;
					}
					/* offset_in_page = 0; */
				}
				*last_data_len = page_size - (valid_data->valid_data_num + 1) *
					sizeof(struct mt_ftl_data_header) - valid_data->valid_data_offset - 4;
				if (*last_data_len > 0) {
					if (last_data_len_src)
						memcpy(&valid_data->valid_buffer[valid_data->valid_data_offset],
							last_data_buffer, *last_data_len);
					else
						memcpy(&valid_data->valid_buffer[valid_data->valid_data_offset],
							(char *)param->gc_page_buffer + offset_in_page,
							*last_data_len);
					valid_header.offset_len = (valid_data->valid_data_offset << 16) | data_len;
					valid_header.sector = sector;
					valid_data->valid_header_offset -= sizeof(struct mt_ftl_data_header);
					mt_ftl_debug("valid_header_offset: %d", valid_data->valid_header_offset);
					memcpy(&valid_data->valid_buffer[valid_data->valid_header_offset],
						&valid_header, sizeof(struct mt_ftl_data_header));
					valid_data->valid_data_num++;
					memcpy(&valid_data->valid_buffer[page_size - 4],
						&valid_data->valid_data_num, 4);
					mt_ftl_updatePMT(dev, cluster, sec_offset, valid_data->info.dst_leb,
						valid_data->info.page_dst * page_size,
						valid_data->valid_data_num - 1, data_len, replay, false);
				}
				mt_ftl_debug("leb:%doffset:%d", valid_data->info.dst_leb, valid_data->info.offset_dst);
				ret = mt_ftl_leb_write(dev, valid_data->info.dst_leb, valid_data->valid_buffer,
					valid_data->info.offset_dst, page_size);
				if (ret)
					goto out;
				if (*last_data_len <= 0) {
					MT_FTL_PARAM_LOCK(dev);
					BIT_UPDATE(param->u4BIT[valid_data->info.dst_leb],
						(*last_data_len + sizeof(struct mt_ftl_data_header)));
					MT_FTL_PARAM_UNLOCK(dev);
					*last_data_len = 0;
				}
				valid_data->valid_data_offset = 0;
				valid_data->valid_data_num = 0;
				memset(valid_data->valid_buffer, 0xFF, page_size * sizeof(char));
				valid_data->info.offset_dst += page_size;
				valid_data->info.page_dst += 1;
			}
			if (*last_data_len > 0) {
				valid_data->valid_data_offset = data_len - *last_data_len;
				if (last_data_len_src)
					memcpy(valid_data->valid_buffer,
						(char *)last_data_buffer + *last_data_len,
						valid_data->valid_data_offset);
				else
					memcpy(valid_data->valid_buffer, (char *)param->gc_page_buffer +
						offset_in_page + *last_data_len, valid_data->valid_data_offset);
				*last_data_len = 0;
			} else {
				mt_ftl_updatePMT(dev, cluster, sec_offset, valid_data->info.dst_leb,
						 valid_data->info.page_dst * page_size,
						 valid_data->valid_data_num, data_len, replay, false);
				valid_header.offset_len = (valid_data->valid_data_offset << 16) | data_len;
				valid_header.sector = sector;
				total_consumed_size = offset_in_page + data_len +
					data_num  * sizeof(struct mt_ftl_data_header) + 4;
				if (total_consumed_size > page_size) {
					last_data_len_src = page_size - offset_in_page -
						(data_num * sizeof(struct mt_ftl_data_header) + 4);
					memcpy(&valid_data->valid_buffer[valid_data->valid_data_offset],
						(char *)param->gc_page_buffer + offset_in_page, last_data_len_src);
					ret = mt_ftl_leb_read(dev, valid_data->info.src_leb,
						param->tmp_page_buffer,
						valid_data->info.offset_src + page_size, page_size);
					if (ret)
						goto out;
					memcpy(&valid_data->valid_buffer[valid_data->valid_data_offset +
						last_data_len_src], (char *)param->tmp_page_buffer,
						data_len - last_data_len_src);
					check_num =
						PAGE_GET_DATA_NUM(param->tmp_page_buffer[(page_size >> 2) - 1]);
					if (check_num == 0x7FFFFFFF) {
						valid_data->info.page_src += 1;
						valid_data->info.offset_src += page_size;
					}
				} else
					memcpy(&valid_data->valid_buffer[valid_data->valid_data_offset],
						(char *)param->gc_page_buffer + offset_in_page,
						data_len);
				valid_data->valid_data_offset += data_len;
				valid_data->valid_header_offset = page_size -
					(valid_data->valid_data_num + 1) * sizeof(struct mt_ftl_data_header) - 4;
				memcpy(&valid_data->valid_buffer[valid_data->valid_header_offset],
						&valid_header, sizeof(struct mt_ftl_data_header));
				valid_data->valid_data_num += 1;
				memcpy(&valid_data->valid_buffer[page_size - 4], &valid_data->valid_data_num, 4);
			}
		}
		if (replay)
			mt_ftl_updatePMT(dev, cluster, sec_offset, valid_data->info.dst_leb,
					 valid_data->info.page_dst * page_size, i,
					 (header_buffer[data_num - i - 1].offset_len & 0xFFFF), replay, false);
	}
	if (replay) {
		data_len = header_buffer[0].offset_len & 0xFFFF;
		offset_in_page = (header_buffer[0].offset_len >> 16) & 0xFFFF;
		total_consumed_size = data_len + offset_in_page + data_num * sizeof(struct mt_ftl_data_header) + 4;
		if (total_consumed_size > page_size) {
			*last_data_len = total_consumed_size - page_size;
		} else if (total_consumed_size < page_size) {
			MT_FTL_PARAM_LOCK(dev);
			BIT_UPDATE(param->u4BIT[valid_data->info.dst_leb], (page_size - total_consumed_size));
			MT_FTL_PARAM_UNLOCK(dev);
		}
	}
out:
	if (last_data_buffer) {
		vfree(last_data_buffer);
		last_data_buffer = NULL;
	}
	return ret;
}

/* The process for data is to get all the sectors in the source block
 * and compare to PMT, if the corresponding block/page/part in PMT is
 * the same as source block/page/part, then current page should be copied
 * destination block, and then update PMT
 */
static int mt_ftl_gc_data(struct mt_ftl_blk *dev, int sleb, int dleb, int *offset_dst, bool replay)
{
	int ret = MT_FTL_SUCCESS;
	u32 data_num = 0, retry = 0;
	u32 data_hdr_offset = 0;
	u32 page_been_read = 0;
	struct mt_ftl_data_header *header_buffer = NULL;
	struct mt_ftl_param *param = dev->param;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_valid_data valid_data;
	int head_data = 0, last_data_len;
	u32 total_consumed_size = 0;
	int page_size = dev->min_io_size;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;

	valid_data.info.offset_src = 0;
	valid_data.info.offset_dst = 0;
	valid_data.info.src_leb = sleb;
	valid_data.info.dst_leb = dleb;
	valid_data.info.page_src = 0;
	valid_data.info.page_dst = 0;
	valid_data.valid_data_num = 0;
	valid_data.valid_data_offset = 0;
	valid_data.valid_header_offset = 0;
	valid_data.valid_buffer = NULL;

	valid_data.valid_buffer = vmalloc(page_size);
	if (!valid_data.valid_buffer) {
		ret = MT_FTL_FAIL;
		ubi_assert(false);
		goto out_free;
	}
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_DATA_READOOB);
	if (!replay)
		mt_ftl_leb_map(dev, dleb);
	ret = ubi_is_mapped(desc, sleb);
	if (ret <= 0 || dleb >= dev->dev_blocks) {
		mt_ftl_err("source_leb(%d) not mapped or dest_leb(%d) over total block", sleb, dleb);
		ret = MT_FTL_FAIL;
		goto out_free;
	}
	if (!replay) {
		ret = mt_ftl_leb_read(dev, sleb, param->gc_page_buffer, valid_data.info.offset_src, page_size);
	} else {
		/* When replay, the data has been copied to dst, so we read dst instead of src
		 * but we still use offset_src to control flow */
		ret = mt_ftl_leb_read(dev, dleb, param->gc_page_buffer, valid_data.info.offset_src, page_size);
	}
	if (ret)
		goto out_free;

	page_been_read = PAGE_BEEN_READ(param->gc_page_buffer[(page_size >> 2) - 1]);
	if (replay && (page_been_read == 0)) {
		mt_ftl_err("End of replay GC Data, offset_src=%d", valid_data.info.offset_src);
		goto out_free;
	}

	data_num = PAGE_GET_DATA_NUM(param->gc_page_buffer[(page_size >> 2) - 1]);
	data_hdr_offset = page_size - (data_num * sizeof(struct mt_ftl_data_header) + 4);
	if ((data_num * sizeof(struct mt_ftl_data_header)) >= page_size) {
		mt_ftl_err("(data_num * sizeof(struct mt_ftl_data_header))(%lx) >= NAND_PAGE_SIZE(%d)",
			   (data_num * sizeof(struct mt_ftl_data_header)), page_size);
		mt_ftl_err("data_hdr_offset = %d, data_num = %d", data_hdr_offset, data_num);
	}
	header_buffer = (struct mt_ftl_data_header *)(&param->gc_page_buffer[data_hdr_offset >> 2]);

	head_data = mt_ftl_check_header(dev, header_buffer, data_num);
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_DATA_READOOB);

	while (head_data && (valid_data.info.offset_src <= max_offset_per_block)) {
		retry = 1;
		last_data_len = 0;
		MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_DATA_READ_UPDATE_PMT);
		ret = mt_ftl_gc_check_data(dev, header_buffer, &valid_data, data_num, &last_data_len, replay);
		if (ret)
			goto out_free;
		MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_DATA_READ_UPDATE_PMT);
		MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_DATA_READOOB);
retry_once:
		if (replay) {
			valid_data.info.offset_dst += page_size;
			valid_data.info.page_dst += 1;
		}

		valid_data.info.offset_src += page_size;
		valid_data.info.page_src += 1;
		/* mt_ftl_err(dev, "offset_src=(%d:%d) offset_dst=(%d:%d), %d", offset_src, offset_src / NAND_PAGE_SIZE,
			*offset_dst, *offset_dst / NAND_PAGE_SIZE, max_offset_per_block); */
		if (valid_data.info.offset_src > max_offset_per_block)
			break;
		if (!replay)
			ret = mt_ftl_leb_read(dev, sleb, param->gc_page_buffer, valid_data.info.offset_src, page_size);
		else
			ret = mt_ftl_leb_read(dev, dleb, param->gc_page_buffer, valid_data.info.offset_src, page_size);

		if (ret)
			goto out_free;

		page_been_read = PAGE_BEEN_READ(param->gc_page_buffer[(page_size >> 2) - 1]);

		data_num = PAGE_GET_DATA_NUM(param->gc_page_buffer[(page_size >> 2) - 1]);
		if (data_num == 0x7FFFFFFF) {
			if (replay && last_data_len && retry) {
				MT_FTL_PARAM_LOCK(dev);
				BIT_UPDATE(param->u4BIT[dleb], (page_size - last_data_len - 4));
				MT_FTL_PARAM_UNLOCK(dev);
				last_data_len = 0;
				retry = 0;
				goto retry_once;
			} else if (retry) {
				retry = 0;
				goto retry_once;
			}
			break;
		}
		data_hdr_offset = page_size - (data_num * sizeof(struct mt_ftl_data_header) + 4);
		if ((data_num * sizeof(struct mt_ftl_data_header)) >= page_size) {
			mt_ftl_err("(data_num * sizeof(struct mt_ftl_data_header))(0x%lx) >= NAND_PAGE_SIZE(%d)",
				   (data_num * sizeof(struct mt_ftl_data_header)), page_size);
			mt_ftl_err("data_hdr_offset = %d, data_num = %d", data_hdr_offset, data_num);
		}
		header_buffer = (struct mt_ftl_data_header *)(&param->gc_page_buffer[data_hdr_offset >> 2]);

		head_data = mt_ftl_check_header(dev, header_buffer, data_num);
		if (replay) {
			if (page_been_read == 0) {
				mt_ftl_err("End of replay GC Data, offset_src = %d", valid_data.info.offset_src);
				goto out_free;
			}
		}
		MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_DATA_READOOB);
	}
	if ((valid_data.valid_data_num || valid_data.valid_data_offset) && (!replay)) {
		if (valid_data.valid_data_num)
			total_consumed_size = valid_data.valid_data_offset +
			valid_data.valid_data_num * sizeof(struct mt_ftl_data_header) + 4;
		else
			total_consumed_size = valid_data.valid_data_offset + 4;
		ret = mt_ftl_leb_write(dev, dleb, valid_data.valid_buffer, valid_data.info.offset_dst, page_size);
		if (ret)
			goto out_free;
		MT_FTL_PARAM_LOCK(dev);
		BIT_UPDATE(param->u4BIT[dleb], (page_size - total_consumed_size));
		MT_FTL_PARAM_UNLOCK(dev);
		valid_data.info.offset_dst += page_size;
		param->u4NextPageOffsetIndicator = 0;
		param->u4DataNum = 0;
		mt_ftl_err("gc finished, valid_data_num=%d", valid_data.valid_data_num);
	}
out_free:
	*offset_dst = valid_data.info.offset_dst;
	if (valid_data.valid_buffer) {
		vfree(valid_data.valid_buffer);
		valid_data.valid_buffer = NULL;
	}
	mt_ftl_debug("u4BIT[%d] = %d", dleb, param->u4BIT[dleb]);

	return ret;
}

static int mt_ftl_gc_findblock(struct mt_ftl_blk *dev, int start_leb, int end_leb, bool ispmt, bool *invalid)
{
	int i = 0;
	int return_leb = MT_INVALID_BLOCKPAGE;
	int max_invalid = 0;
	int total_invalid = dev->leb_size - (dev->leb_size / dev->min_io_size * 4);
	struct mt_ftl_param *param = dev->param;

	if (start_leb > dev->dev_blocks || end_leb > dev->dev_blocks) {
		mt_ftl_err("u4StartLeb(%d)u4EndLeb(%d) is larger than NAND_TOTAL_BLOCK_NUM(%d)",
			start_leb, end_leb, dev->dev_blocks);
		return MT_FTL_FAIL;
	}
	if (unlikely(ispmt))
		total_invalid = dev->leb_size;
	MT_FTL_PARAM_LOCK(dev);
	for (i = start_leb; i < end_leb; i++) {
		if (param->u4BIT[i] >= total_invalid) {
			return_leb = i;
			*invalid = true;
			MT_FTL_PARAM_UNLOCK(dev);
			mt_ftl_debug("u4BIT[%d] = %d", return_leb, param->u4BIT[return_leb]);
			return return_leb;
		}

		if (param->u4BIT[i] > max_invalid) {
			max_invalid = param->u4BIT[i];
			return_leb = i;
			mt_ftl_debug("BIT[%d] = %d(%d:%d)", return_leb, param->u4BIT[return_leb], start_leb, end_leb);
		}
	}

	*invalid = false;
	MT_FTL_PARAM_UNLOCK(dev);
	return return_leb;
}

static int mt_ftl_check_replay_commit(struct mt_ftl_blk *dev, int leb)
{
	int i, commit = 0;
	struct mt_ftl_param *param = dev->param;

	for (i = 0; i < param->replay_blk_index; i++) {
		if (leb == param->replay_blk_rec[i]) {
			mt_ftl_err("u4SrcInvalidLeb(%d)==replay_blk_rec[%d](%d)", leb, i, param->replay_blk_rec[i]);
			commit = 1;
			break;
		}
	}
	return commit;
}

static int mt_ftl_write_replay_blk(struct mt_ftl_blk *dev, int leb, bool replay)
{
	int ret = MT_FTL_SUCCESS;
	struct mt_ftl_param *param = dev->param;

	if (!replay) {
		param->replay_page_buffer[0] = MT_MAGIC_NUMBER;
		param->replay_page_buffer[1] = leb;
		mt_ftl_err("u4NextReplayOffsetIndicator=%d", param->u4NextReplayOffsetIndicator);
		ret = mt_ftl_leb_write(dev, REPLAY_BLOCK, param->replay_page_buffer, param->u4NextReplayOffsetIndicator,
			 dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		param->u4NextReplayOffsetIndicator += dev->min_io_size;
	}
	param->replay_blk_rec[param->replay_blk_index] = leb;
	param->replay_blk_index++;
	return ret;
}

static int mt_ftl_gc(struct mt_ftl_blk *dev, int *updated_page, bool ispmt, bool replay, int *commit)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int src_leb = MT_INVALID_BLOCKPAGE;
	int return_leb = 0, offset_dst = 0;
	bool invalid = false, need = true;
	int start_leb = DATA_START_BLOCK, end_leb = dev->dev_blocks;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_FINDBLK);
	/* Get the most invalid block */
	if (unlikely(ispmt)) {
		start_leb = PMT_START_BLOCK;
		end_leb = DATA_START_BLOCK;
	}
	src_leb = mt_ftl_gc_findblock(dev, start_leb, end_leb, ispmt, &invalid);
	if (invalid)
		goto gc_end;
	if (((u32)src_leb) == MT_INVALID_BLOCKPAGE) {
		mt_ftl_err("cannot find block for GC, isPMT = %d", ispmt);
		return MT_FTL_FAIL;
	}

	/* mt_ftl_err(dev, "u4BIT[%d] = %d, u4GCReservePMTLeb = 0x%x, u4GCReserveLeb = 0x%x",
			u4SrcInvalidLeb, param->u4BIT[u4SrcInvalidLeb],
			param->u4GCReservePMTLeb, param->u4GCReserveLeb);	 Temporary */
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_FINDBLK);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_REMAP);
	if (!replay && !ispmt) {
		ret = mt_ftl_check_replay_commit(dev, src_leb);
		if (ret) {
			mt_ftl_err("commit src_leb(%d) == replay_blk_rec", src_leb);
			mt_ftl_commit(dev);
			*commit = 0;
			need = false;
		}
	}
	if (!ispmt) {
		ret = mt_ftl_write_replay_blk(dev, param->u4GCReserveLeb, replay);
		if (ret)
			return ret;
	}
	/* Call sub function for pmt/data */
	if (unlikely(ispmt))
		ret = mt_ftl_gc_pmt(dev, src_leb, param->u4GCReservePMTLeb, &offset_dst, replay);
	else
		ret = mt_ftl_gc_data(dev, src_leb, param->u4GCReserveLeb, &offset_dst, replay);
	/* PMT not in PMTBlock but dirty, ret == 1 is u4SrcInvalidLeb mapped*/
	if (unlikely(ispmt && (ret == 1)))
		goto gc_end;
	else if (ret) {
		mt_ftl_err("GC sub function failed, u4SrcInvalidLeb = %d, offset_dst=%d, ret = 0x%x",
			   src_leb, offset_dst, ret);
		return MT_FTL_FAIL;
	}
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_CPVALID);
gc_end:
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GC_REMAP);
	if (!replay && !ispmt) {
		if ((page_num_per_block - 1) == param->replay_blk_index && need) {
				mt_ftl_err("NAND_PAGE_NUM_PER_BLOCK(%d) <= param->replay_blk_index(%d)",
					   page_num_per_block, param->replay_blk_index);
				*commit = 1;
		}
	}

	/* TODO: Use this information for replay, instead of check MT_PAGE_HAD_BEEN_READ */
	*updated_page = offset_dst / dev->min_io_size;
	if (*updated_page == page_num_per_block) {
		mt_ftl_err("There is no more free pages in the gathered block");
		mt_ftl_err("desc->vol->ubi->volumes[%d]->reserved_pebs = %d",
			   dev->vol_id, dev->dev_blocks);
		for (i = PMT_START_BLOCK; i < dev->dev_blocks; i += 8) {
			mt_ftl_err("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d", param->u4BIT[i],
				   param->u4BIT[i + 1], param->u4BIT[i + 2], param->u4BIT[i + 3],
				   param->u4BIT[i + 4], param->u4BIT[i + 5], param->u4BIT[i + 6],
				   param->u4BIT[i + 7]);
		}
		return MT_FTL_FAIL;
	}

	if (unlikely(ispmt)) {
		return_leb = param->u4GCReservePMTLeb;
		param->u4GCReservePMTLeb = src_leb;
		mt_ftl_err("u4GCReservePMTLeb = %d, *updated_page = %d, u4GCReserveLeb = %d",
				param->u4GCReservePMTLeb, *updated_page, param->u4GCReserveLeb);
		/* Temporary */
		if (param->u4GCReservePMTLeb >= dev->dev_blocks)
			mt_ftl_err("param->u4GCReservePMTLeb(%d) is larger than NAND_TOTAL_BLOCK_NUM(%d)",
				   param->u4GCReservePMTLeb, dev->dev_blocks);
		MT_FTL_PARAM_LOCK(dev);
		param->u4BIT[param->u4GCReservePMTLeb] = 0;
		MT_FTL_PARAM_UNLOCK(dev);
		mt_ftl_debug("u4BIT[%d] = %d", param->u4GCReservePMTLeb, param->u4BIT[param->u4GCReservePMTLeb]);
	} else {
		return_leb = param->u4GCReserveLeb;
		param->u4GCReserveLeb = src_leb;
		mt_ftl_err("u4GCReserveLeb = %d, *updated_page = %d", param->u4GCReserveLeb, *updated_page);
		if (param->u4GCReserveLeb >= dev->dev_blocks)
			mt_ftl_err("param->u4GCReserveLeb(%d) is larger than NAND_TOTAL_BLOCK_NUM(%d)",
				   param->u4GCReserveLeb, dev->dev_blocks);
		MT_FTL_PARAM_LOCK(dev);
		param->u4BIT[param->u4GCReserveLeb] = 0;
		MT_FTL_PARAM_UNLOCK(dev);
		mt_ftl_debug("u4BIT[%d] = %d", param->u4GCReserveLeb, param->u4BIT[param->u4GCReserveLeb]);
	}

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GC_REMAP);
	return return_leb;
}

static int mt_ftl_leb_lastpage_offset(struct mt_ftl_blk *dev, int leb)
{
	int ret = MT_FTL_SUCCESS;
	int offset = 0;
	struct mt_ftl_param *param = dev->param;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;

	while (offset <= max_offset_per_block) {
		ret = mt_ftl_leb_read(dev, leb, param->tmp_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		if (param->tmp_page_buffer[0] == 0xFFFFFFFF) {
			/* mt_ftl_err(dev, "[INFO] Get last page in leb:%d, page:%d", leb, offset / NAND_PAGE_SIZE); */
			break;
		}
		offset += dev->min_io_size;
	}
	return offset;
}

/* TODO: Add new block to replay block, but have to consider the impact of original valid page to replay */
static int mt_ftl_getfreeblock(struct mt_ftl_blk *dev, int *updated_page, bool ispmt, bool replay)
{
	int ret = MT_FTL_SUCCESS;
	int free_leb = 0;
	int commit = 0;
	struct mt_ftl_param *param = dev->param;
	/* const int max_offset_per_block = dev->leb_size - dev->min_io_size; */
	/* const int page_num_per_block = dev->leb_size / dev->min_io_size; */

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GETFREEBLOCK_GETLEB);
	if (ispmt) {
		if (param->u4NextFreePMTLebIndicator != MT_INVALID_BLOCKPAGE) {
			free_leb = param->u4NextFreePMTLebIndicator;
			if (!replay)
				mt_ftl_leb_map(dev, free_leb);
			*updated_page = 0;
			param->u4NextFreePMTLebIndicator++;
			mt_ftl_debug("u4NextFreePMTLebIndicator=%d", param->u4NextFreePMTLebIndicator);
			/* The PMT_BLOCK_NUM + 2 block is reserved to param->u4GCReserveLeb */
			if (param->u4NextFreePMTLebIndicator >= DATA_START_BLOCK - 1) {
				/* mt_ftl_err(dev, "[INFO] u4NextFreePMTLebIndicator is in the end"); */
				param->u4NextFreePMTLebIndicator = MT_INVALID_BLOCKPAGE;
				mt_ftl_debug("u4NextFreePMTLebIndicator=%d", param->u4NextFreePMTLebIndicator);
			}
		} else {
			free_leb = mt_ftl_gc(dev, updated_page, ispmt, replay, &commit);
			if (!replay)
				mt_ftl_leb_remap(dev, param->u4GCReservePMTLeb);
		}

		MT_FTL_PROFILE_END(MT_FTL_PROFILE_GETFREEBLOCK_GETLEB);
		return free_leb;
	}

	if (param->u4NextFreeLebIndicator != MT_INVALID_BLOCKPAGE) {
		free_leb = param->u4NextFreeLebIndicator;
		*updated_page = 0;
		if (!replay)
			mt_ftl_leb_map(dev, free_leb);
		param->u4NextFreeLebIndicator++;
		param->u4NextPageOffsetIndicator = 0;
		mt_ftl_debug("u4NextFreeLebIndicator=%d", param->u4NextFreeLebIndicator);
		/* The last block is reserved to param->u4GCReserveLeb */
		/* [Bean] Please reserved block for UBI WL 2 PEB*/
		if (param->u4NextFreeLebIndicator >= dev->dev_blocks - 1) {
			param->u4NextFreeLebIndicator = MT_INVALID_BLOCKPAGE;
			mt_ftl_err("u4NextFreeLebIndicator=%d", param->u4NextFreeLebIndicator);
		}
		ret = mt_ftl_write_replay_blk(dev, free_leb, replay);
		if (ret)
			return MT_FTL_FAIL;
	} else
		free_leb = mt_ftl_gc(dev, updated_page, ispmt, replay, &commit);

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GETFREEBLOCK_GETLEB);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_GETFREEBLOCK_PUTREPLAY_COMMIT);
	ret = mt_ftl_write_replay_blk(dev, free_leb, replay);
	if (ret)
		return MT_FTL_FAIL;
	if (!replay) {
		if (commit) {
			PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, free_leb, *updated_page);
			mt_ftl_err("commit u4NextLebPageIndicator=0x%x", param->u4NextLebPageIndicator);
			mt_ftl_commit(dev);
		}
		if (param->u4NextFreeLebIndicator == MT_INVALID_BLOCKPAGE)
			mt_ftl_leb_remap(dev, param->u4GCReserveLeb);
	}

	mt_ftl_err("u4FreeLeb = %d, u4NextFreeLebIndicator = %d, isPMT = %d, updated_page = %d",
		   free_leb, param->u4NextFreeLebIndicator, ispmt, *updated_page);
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_GETFREEBLOCK_PUTREPLAY_COMMIT);

	return free_leb;
}

/* sync *page_buffer to the end of dst_leb */
static int mt_ftl_write_to_blk(struct mt_ftl_blk *dev, int dst_leb, u32 *page_buffer)
{
	int offset = 0;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;

	mt_ftl_leb_map(dev, dst_leb);
	offset = mt_ftl_leb_lastpage_offset(dev, dst_leb);
	if (offset > max_offset_per_block) {
		mt_ftl_leb_remap(dev, dst_leb);
		offset = 0;
	}

	mt_ftl_debug("dst_leb=%d, offset=%d, page_buffer[0]=0x%x", dst_leb, offset, page_buffer[0]);
	return mt_ftl_leb_write(dev, dst_leb, page_buffer, offset, dev->min_io_size);
}

static int mt_ftl_write_page(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int leb = 0, page = 0, cache_num = 0;
	sector_t sector = 0;
	u32 cluster = 0, sec_offset = 0;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;
	u32 data_hdr_offset = 0;
	int data_offset = 0, data_len = 0;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_PAGE_WRITEOOB);
	leb = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4NextLebPageIndicator);
	page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4NextLebPageIndicator);

	mt_ftl_debug("u4NextLebPageIndicator = 0x%x, leb = %d, page = %d", param->u4NextLebPageIndicator, leb, page);
	/* write the last data of page, but cannot write into the page, write to next */
	if (param->u4DataNum == 0) {
		ret = mt_ftl_leb_write(dev, leb, param->u1DataCache, page * dev->min_io_size, dev->min_io_size);
		if (ret)
			return ret;
		MT_FTL_PARAM_LOCK(dev);
		BIT_UPDATE(param->u4BIT[leb], (dev->min_io_size - param->u4NextLebPageIndicator - 4));
		MT_FTL_PARAM_UNLOCK(dev);
		mt_ftl_err("data_num==0, BIT[%d]=%d, page=%d, size=%d", leb, param->u4BIT[leb], page,
			(dev->min_io_size - param->u4NextLebPageIndicator));
		param->u4NextLebPageIndicator = 0;
		goto next_indicator;
	}
	data_hdr_offset = dev->min_io_size - (param->u4DataNum * sizeof(struct mt_ftl_data_header) + 4);
	if ((data_hdr_offset + param->u4DataNum * sizeof(struct mt_ftl_data_header)) >=
	    dev->min_io_size) {
		mt_ftl_err("(data_hdr_offset+u4DataNum*sizeof(mt_ftl_data_header))(0x%lx)>=NAND_PAGE_SIZE(%d)"
			   , (data_hdr_offset + param->u4DataNum * sizeof(struct mt_ftl_data_header)),
			   dev->min_io_size);
		mt_ftl_err("data_hdr_offset = %d, param->u4DataNum = %d", data_hdr_offset, param->u4DataNum);
	}
	ubi_assert(param->u4DataNum < MTKFTL_MAX_DATA_NUM_PER_PAGE);
	memcpy(&param->u1DataCache[data_hdr_offset],
	       &param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum],
	       param->u4DataNum * sizeof(struct mt_ftl_data_header));
	memcpy(&param->u1DataCache[dev->min_io_size - 4], &param->u4DataNum, 4);
	ret = mt_ftl_leb_write(dev, leb, param->u1DataCache, page * dev->min_io_size, dev->min_io_size);
	if (ret)
		return ret;
	data_offset = (param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum].offset_len >> 16) & 0xFFFF;
	data_len = param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum].offset_len & 0xFFFF;
	if ((data_offset + data_len) < data_hdr_offset) {
		MT_FTL_PARAM_LOCK(dev);
		BIT_UPDATE(param->u4BIT[leb], (data_hdr_offset - data_offset - data_len));
		MT_FTL_PARAM_UNLOCK(dev);
	}
	param->u4NextPageOffsetIndicator = 0;
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_PAGE_WRITEOOB);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_PAGE_GETFREEBLK);
	for (i = MTKFTL_MAX_DATA_NUM_PER_PAGE - 1;
	     i >= MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum; i--) {
		sector = param->u4Header[i].sector;
		cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / dev->pm_per_io;
		sec_offset =
		    ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (dev->pm_per_io - 1);
		ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
		if (PMT_INDICATOR_IS_INCACHE(param->u4PMTIndicator[cluster])) {
			cache_num = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[cluster]);
			ubi_assert(cache_num < PMT_CACHE_NUM);
			ubi_assert(sec_offset < dev->pm_per_io);
			PMT_RESET_DATA_INCACHE(param->u4MetaPMTCache[cache_num * dev->pm_per_io +
								     sec_offset]);
		}
	}
next_indicator:
	page++;
	if (page == page_num_per_block) {
		leb = mt_ftl_getfreeblock(dev, &page, false, false);
		if (((u32)leb) == MT_INVALID_BLOCKPAGE) {
			mt_ftl_err("mt_ftl_getfreeblock failed");
			return MT_FTL_FAIL;
		}
	}
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_PAGE_GETFREEBLK);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_PAGE_RESET);
	memset(param->u1DataCache, 0xFF, dev->min_io_size * sizeof(unsigned char));
	memset(param->u4Header, 0xFF, MTKFTL_MAX_DATA_NUM_PER_PAGE * sizeof(struct mt_ftl_data_header));
	param->u4DataNum = 0;

	PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, leb, page);
	/*mt_ftl_err(dev, "u4NextLebPageIndicator = 0x%x", param->u4NextLebPageIndicator);*/

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_PAGE_RESET);

	return ret;
}

int mt_ftl_commitPMT(struct mt_ftl_blk *dev, bool replay, bool commit)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int pmt_block = 0, pmt_page = 0;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_COMMIT_PMT);
	if (((!replay) && commit && (param->u4DataNum != 0)) || param->u4NextPageOffsetIndicator)
		ret = mt_ftl_write_page(dev);

	for (i = 0; i < PMT_CACHE_NUM; i++) {
		if (param->i4CurrentPMTClusterInCache[i] == 0xFFFFFFFF)
			continue;
		ubi_assert(param->i4CurrentPMTClusterInCache[i] <= PMT_TOTAL_CLUSTER_NUM);
		if (!PMT_INDICATOR_IS_INCACHE
		    (param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]])) {
			mt_ftl_err("i4CurrentPMTClusterInCache(%d) is not in cache",
				   param->i4CurrentPMTClusterInCache[i]);
			return MT_FTL_FAIL;
		}
		if (!PMT_INDICATOR_IS_DIRTY
		    (param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]])) {
			mt_ftl_err("u4PMTIndicator[%d]=0x%x is not dirty", param->i4CurrentPMTClusterInCache[i],
				   param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
			/* clear i4CurrentPMTClusterInCache */
			PMT_INDICATOR_RESET_INCACHE(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
			param->i4CurrentPMTClusterInCache[i] = 0xFFFFFFFF;
			continue;
		}
		/* Update param->u4BIT of the block that is originally in param->u4PMTCache */
		pmt_block = PMT_INDICATOR_GET_BLOCK(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
		pmt_page = PMT_INDICATOR_GET_PAGE(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
		/*mt_ftl_err(dev, "i4CurrentPMTClusterInCache = %d, u4PMTIndicator[i4CurrentPMTClusterInCache] = 0x%x",
		   param->i4CurrentPMTClusterInCache, param->u4PMTIndicator[param->i4CurrentPMTClusterInCache]);
		   mt_ftl_err(dev, "pmt_block = %d, pmt_page = %d", pmt_block, pmt_page);       // Temporary */
		ubi_assert(pmt_block < dev->dev_blocks);
		if ((((u32)pmt_block) != MT_INVALID_BLOCKPAGE) && (pmt_block != 0)) {
			MT_FTL_PARAM_LOCK(dev);
			BIT_UPDATE(param->u4BIT[pmt_block], (dev->min_io_size * 2));
			MT_FTL_PARAM_UNLOCK(dev);
			mt_ftl_debug("u4BIT[%d] = %d", pmt_block, param->u4BIT[pmt_block]);
		}

		/* Calculate new block/page from param->u4CurrentPMTLebPageIndicator
		 * Update param->u4CurrentPMTLebPageIndicator
		 * and check the correctness of param->u4CurrentPMTLebPageIndicator
		 * and if param->u4CurrentPMTLebPageIndicator is full
		 * need to call get free block/page function
		 * Write old param->u4PMTCache back to new block/page
		 * Update param->u4PMTIndicator of the block/page that is originally in param->u4PMTCache
		 */

		pmt_block = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4CurrentPMTLebPageIndicator);
		pmt_page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4CurrentPMTLebPageIndicator);
		/* mt_ftl_err("u4CurrentPMTLebPageIndicator = 0x%x", param->u4CurrentPMTLebPageIndicator); */
		/* check replay conditions */
		switch (last_replay_flag) {
		case REPLAY_LAST_BLK:
			mt_ftl_err("case 1 REPLAY_LAST_BLK");
			if (recorded_pmt_blk != pmt_block) {
				ret = mt_ftl_leb_recover(dev, pmt_block, pmt_page * dev->min_io_size, 0);
				if (ret == MT_FTL_FAIL) {
					mt_ftl_err("[Bean]leb(%d) recover fail", pmt_block);
					return MT_FTL_FAIL;
				} else if (ret == 1) {
					MT_FTL_PARAM_LOCK(dev);
					recorded_pmt_blk = pmt_block;
					last_replay_flag = REPLAY_FREE_BLK;
					MT_FTL_PARAM_UNLOCK(dev);
				}
			}
			break;
		case REPLAY_CLEAN_BLK:
			mt_ftl_err("case 2 REPLAY_CLEAN_BLK");
			MT_FTL_PARAM_LOCK(dev);
			recorded_pmt_blk = pmt_block;
			MT_FTL_PARAM_UNLOCK(dev);
			break;
		}
		if (!replay || (recorded_pmt_blk == pmt_block)) {
			if (ubi_is_mapped(desc, pmt_block)) {
				if (i >= PMT_CACHE_NUM)
					mt_ftl_err("i(%d) is larger than PMT_CACHE_NUM(%d)", i, PMT_CACHE_NUM);
				mt_ftl_err("write pmt %d %d", pmt_block, pmt_page);
				ret = mt_ftl_leb_write(dev, pmt_block, &param->u4PMTCache[i * dev->pm_per_io],
						 pmt_page * dev->min_io_size, dev->min_io_size);
				ret = mt_ftl_leb_write(dev, pmt_block, &param->u4MetaPMTCache[i * dev->pm_per_io],
						 (pmt_page + 1) * dev->min_io_size, dev->min_io_size);
			} else {
				mt_ftl_err("pmt_block(%d) is not mapped", pmt_block);
				return MT_FTL_FAIL;
			}
		}
		PMT_INDICATOR_SET_BLOCKPAGE(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]],
					pmt_block, pmt_page, 0, i);
		mt_ftl_debug("u4PMTIndicator[%d] = 0x%x", param->i4CurrentPMTClusterInCache[i],
			   param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
		/* clear i4CurrentPMTClusterInCache */
		if (param->i4CurrentReadPMTClusterInCache == param->i4CurrentPMTClusterInCache[i]) {
			mt_ftl_err("[Bean]clear read PMTCluster cache");
			param->i4CurrentReadPMTClusterInCache = 0xFFFFFFFF;
		}
		PMT_INDICATOR_RESET_INCACHE(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
		param->i4CurrentPMTClusterInCache[i] = 0xFFFFFFFF;
		pmt_page += 2;
		if (pmt_page >= page_num_per_block) {
			pmt_block = mt_ftl_getfreeblock(dev, &pmt_page, true, replay);
			if (((u32)pmt_block) == MT_INVALID_BLOCKPAGE) {
				mt_ftl_err("mt_ftl_getfreeblock failed");
				return MT_FTL_FAIL;
			}
		}
		PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4CurrentPMTLebPageIndicator, pmt_block, pmt_page);
	}
	mt_ftl_err("commit pmt end 0x%x", param->u4CurrentPMTLebPageIndicator);
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_COMMIT_PMT);
	return ret;
}

int mt_ftl_commit_indicators(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS;
	int index = 0;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	memset(param->commit_page_buffer, 0, (dev->min_io_size >> 2) * sizeof(unsigned int));
	param->commit_page_buffer[0] = MT_MAGIC_NUMBER;
	/* param->commit_page_buffer[1] = param->u4NextPageOffsetIndicator; */
	/* param->commit_page_buffer[1] = param->u4NextReplayOffsetIndicator; */
	param->commit_page_buffer[1] = 0;
	param->commit_page_buffer[2] = param->u4NextLebPageIndicator;
	param->commit_page_buffer[3] = param->u4CurrentPMTLebPageIndicator;
	param->commit_page_buffer[4] = param->u4NextFreeLebIndicator;
	param->commit_page_buffer[5] = param->u4NextFreePMTLebIndicator;
	param->commit_page_buffer[6] = param->u4GCReserveLeb;
	param->commit_page_buffer[7] = param->u4GCReservePMTLeb;
	index = 8;
	if (index + PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("index + PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   index + PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, PMT_TOTAL_CLUSTER_NUM = %d", index, PMT_TOTAL_CLUSTER_NUM);
	}
	memcpy(&param->commit_page_buffer[index], param->u4PMTIndicator, PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int));
	index += ((PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int)) >> 2);
	if (index + NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("index + NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   index + NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, NAND_TOTAL_BLOCK_NUM = %d", index, NAND_TOTAL_BLOCK_NUM);
	}
	memcpy(&param->commit_page_buffer[index], param->u4BIT, NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int));
	index += ((NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int)) >> 2);
	if (index + PMT_CACHE_NUM * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("index + PMT_CACHE_NUM * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   index + PMT_CACHE_NUM * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, PMT_CACHE_NUM = %d", index, PMT_CACHE_NUM);
	}
	memcpy(&param->commit_page_buffer[index], param->i4CurrentPMTClusterInCache,
		PMT_CACHE_NUM * sizeof(unsigned int));

	mt_ftl_err("u4NextPageOffsetIndicator=0x%x", param->u4NextPageOffsetIndicator);
	mt_ftl_err("u4NextReplayOffsetIndicator=0x%x", param->u4NextReplayOffsetIndicator);
	mt_ftl_err("u4NextLebPageIndicator=0x%x", param->u4NextLebPageIndicator);
	mt_ftl_err("u4CurrentPMTLebPageIndicator=0x%x", param->u4CurrentPMTLebPageIndicator);
	mt_ftl_err("u4NextFreeLebIndicator=0x%x", param->u4NextFreeLebIndicator);
	mt_ftl_err("u4NextFreePMTLebIndicator=0x%x", param->u4NextFreePMTLebIndicator);
	mt_ftl_err("u4GCReserveLeb=0x%x", param->u4GCReserveLeb);
	mt_ftl_err("u4GCReservePMTLeb=0x%x", param->u4GCReservePMTLeb);
	mt_ftl_err("u4PMTIndicator=0x%x, 0x%x, 0x%x, 0x%x", param->u4PMTIndicator[0],
		param->u4PMTIndicator[1], param->u4PMTIndicator[2], param->u4PMTIndicator[3]);
	mt_ftl_err("u4BIT=0x%x,0x%x,0x%x,0x%x", param->u4BIT[0], param->u4BIT[1], param->u4BIT[2], param->u4BIT[3]);
	mt_ftl_err("i4CurrentPMTClusterInCache = 0x%x, 0x%x, 0x%x, 0x%x",
		   param->i4CurrentPMTClusterInCache[0], param->i4CurrentPMTClusterInCache[1],
		   param->i4CurrentPMTClusterInCache[2], param->i4CurrentPMTClusterInCache[3]);

	mt_ftl_write_to_blk(dev, CONFIG_START_BLOCK, param->commit_page_buffer);
	mt_ftl_write_to_blk(dev, CONFIG_START_BLOCK + 1, param->commit_page_buffer);

	mt_ftl_leb_remap(dev, REPLAY_BLOCK);
	memset(param->replay_blk_rec, 0xFF, page_num_per_block * sizeof(unsigned int));
	param->u4NextReplayOffsetIndicator = 0;
	param->replay_blk_index = 0;
	return ret;
}

int mt_ftl_commit(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	struct mt_ftl_param *param = dev->param;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_COMMIT);
	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK);
	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK + 1);

	mt_ftl_commitPMT(dev, false, true);
	for (i = 0; i < PMT_CACHE_NUM; i++) {
		if (param->i4CurrentPMTClusterInCache[i] == 0xFFFFFFFF)
			continue;
		ubi_assert(param->i4CurrentPMTClusterInCache[i] <= PMT_TOTAL_CLUSTER_NUM);
		if (!PMT_INDICATOR_IS_INCACHE
		    (param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]])) {
			mt_ftl_err("i4CurrentPMTClusterInCache(%d)not in cache", param->i4CurrentPMTClusterInCache[i]);
			return MT_FTL_FAIL;
		}

		PMT_INDICATOR_RESET_INCACHE(param->u4PMTIndicator
					    [param->i4CurrentPMTClusterInCache[i]]);
		mt_ftl_err("u4PMTIndicator[%d] = %d", param->i4CurrentPMTClusterInCache[i],
			   param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
		param->i4CurrentPMTClusterInCache[i] = 0xFFFFFFFF;
	}

	/* Force to store param->u1DataCache into flash */
	/* if (param->u4DataNum)
		mt_ftl_write_page(dev); */

	ret = mt_ftl_commit_indicators(dev);

	ubi_sync(dev->ubi_num);

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_COMMIT);
	return ret;
}

static int mt_ftl_downloadPMT(struct mt_ftl_blk *dev, u32 cluster, int cache_num)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int pmt_block = 0, pmt_page = 0;

	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;

	ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
	pmt_block = PMT_INDICATOR_GET_BLOCK(param->u4PMTIndicator[cluster]);
	pmt_page = PMT_INDICATOR_GET_PAGE(param->u4PMTIndicator[cluster]);
	if (param->u4MetaPMTCache[0] == 0xFFFFFFFF) {
		mt_ftl_err("pmt_block=%d,cache_num=%d,cluster=%d,u4PMTCache[0]=0x%x,u4MetaPMTCache[0]=0x%x",
			   pmt_block, cache_num, cluster, param->u4PMTCache[0], param->u4MetaPMTCache[0]);
	}
	ubi_assert(cache_num < PMT_CACHE_NUM);

	if (unlikely(pmt_block == 0) || unlikely(ubi_is_mapped(desc, pmt_block) == 0)) {
		mt_ftl_err("unlikely pmt_block %d", ubi_is_mapped(desc, pmt_block));
		memset(&param->u4PMTCache[cache_num * dev->pm_per_io], 0xFF,
		       dev->pm_per_io * sizeof(unsigned int));
		memset(&param->u4MetaPMTCache[cache_num * dev->pm_per_io], 0xFF,
		       dev->pm_per_io * sizeof(unsigned int));
	} else {
		ret = mt_ftl_leb_read(dev, pmt_block, &param->u4PMTCache[cache_num * dev->pm_per_io],
				pmt_page * dev->min_io_size , dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		ret = mt_ftl_leb_read(dev, pmt_block, &param->u4MetaPMTCache[cache_num * dev->pm_per_io],
			(pmt_page + 1) * dev->min_io_size, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
	}
	/* consider cluser if in cache */
	for (i = 0; i < PMT_CACHE_NUM; i++) {
		if (param->i4CurrentPMTClusterInCache[i] == cluster) {
			mt_ftl_err("[Bean]Tempory solution cluster is in cache already(%d)\n", i);
			dump_stack();
			break;
		}
	}
	param->i4CurrentPMTClusterInCache[cache_num] = cluster;
	mt_ftl_debug("i4CurrentPMTClusterInCache[%d]=%d", cache_num, param->i4CurrentPMTClusterInCache[cache_num]);
	PMT_INDICATOR_SET_CACHE_BUF_NUM(param->u4PMTIndicator[cluster], cache_num);
	mt_ftl_debug("u4PMTIndicator[%d] = 0x%x", cluster, param->u4PMTIndicator[cluster]);

	return ret;
}

int mt_ftl_updatePMT(struct mt_ftl_blk *dev, u32 cluster, int sec_offset, int leb, int offset,
		     int part, u32 cmpr_data_size, bool replay, bool commit)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	u32 *pmt = NULL;
	u32 *meta_pmt = NULL;
	int old_leb = 0, old_data_size = 0;
	struct mt_ftl_param *param = dev->param;

	ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
	if (!PMT_INDICATOR_IS_INCACHE(param->u4PMTIndicator[cluster])) { /* cluster is not in cache */
		MT_FTL_PROFILE_START(MT_FTL_PROFILE_UPDATE_PMT_FINDCACHE_COMMITPMT);
		for (i = 0; i < PMT_CACHE_NUM; i++) {
			if (param->i4CurrentPMTClusterInCache[i] == 0xFFFFFFFF)
				break;
			ubi_assert(param->i4CurrentPMTClusterInCache[i] <= PMT_TOTAL_CLUSTER_NUM);
			if (!PMT_INDICATOR_IS_INCACHE(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]])) {
				/* Cluster download PMT CLUSTER cache, but i4CurrentPMTClusterInCache not to update */
				mt_ftl_err("i4CurrentPMTClusterInCache (%d) is not in cache",
					   param->i4CurrentPMTClusterInCache[i]);
				dump_stack();
				return MT_FTL_FAIL;
			}
			if (!PMT_INDICATOR_IS_DIRTY(param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]))
				break;
		}
		if (i == PMT_CACHE_NUM) {
			mt_ftl_err("All PMT cache are dirty, start commit PMT");
			/* Just for downloading corresponding PMT in cache */
			if ((((u32)leb) == MT_INVALID_BLOCKPAGE) || (!commit))
				mt_ftl_commitPMT(dev, replay, false);
			else
				/* mt_ftl_commit(dev); */
				mt_ftl_commitPMT(dev, replay, true);
			i = 0;
		}
		if (param->i4CurrentPMTClusterInCache[i] != 0xFFFFFFFF) {
			PMT_INDICATOR_RESET_INCACHE(param->u4PMTIndicator
						    [param->i4CurrentPMTClusterInCache[i]]);
			mt_ftl_err("Reset (%d) u4PMTIndicator[%d]=0x%x", i, param->i4CurrentPMTClusterInCache[i],
				   param->u4PMTIndicator[param->i4CurrentPMTClusterInCache[i]]);
			param->i4CurrentPMTClusterInCache[i] = 0xFFFFFFFF;
		}
		MT_FTL_PROFILE_END(MT_FTL_PROFILE_UPDATE_PMT_FINDCACHE_COMMITPMT);
		MT_FTL_PROFILE_START(MT_FTL_PROFILE_UPDATE_PMT_DOWNLOADPMT);

		/* Download PMT from the block/page in param->u4PMTIndicator[cluster] */
		ret = mt_ftl_downloadPMT(dev, cluster, i);
		if (ret)
			return ret;

		MT_FTL_PROFILE_END(MT_FTL_PROFILE_UPDATE_PMT_DOWNLOADPMT);
	} else
		i = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[cluster]);

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_UPDATE_PMT_MODIFYPMT);
	if (((u32)leb) == MT_INVALID_BLOCKPAGE) {	/* Just for downloading corresponding PMT in cache */
		MT_FTL_PROFILE_END(MT_FTL_PROFILE_UPDATE_PMT_MODIFYPMT);
		return ret;
	}

	ubi_assert(i < PMT_CACHE_NUM);
	ubi_assert(sec_offset < dev->pm_per_io);

	pmt = &param->u4PMTCache[i * dev->pm_per_io + sec_offset];
	meta_pmt = &param->u4MetaPMTCache[i * dev->pm_per_io + sec_offset];

	/* Update param->u4BIT */
	if (*pmt != NAND_DEFAULT_VALUE) {
		old_leb = PMT_GET_BLOCK(*pmt);
		old_data_size = PMT_GET_DATASIZE(*meta_pmt);
		ubi_assert(old_leb < dev->dev_blocks);
		MT_FTL_PARAM_LOCK(dev);
		BIT_UPDATE(param->u4BIT[old_leb], (old_data_size + sizeof(struct mt_ftl_data_header)));
		MT_FTL_PARAM_UNLOCK(dev);
		if (old_data_size == 0) {
			mt_ftl_err("pmt = 0x%x, meta_pmt = 0x%x, u4PMTIndicator[%d] = 0x%x",
				   *pmt, *meta_pmt, cluster, param->u4PMTIndicator[cluster]);
		}
		if ((param->u4BIT[old_leb] & 0x3FFFF) == 0)
			mt_ftl_err("u4BIT[%d] = %d", old_leb, param->u4BIT[old_leb]);
	}

	/* Update param->u4PMTCache and param->u4MetaPMTCache */
	PMT_SET_BLOCKPAGE(*pmt, leb, offset / dev->min_io_size);
	META_PMT_SET_DATA(*meta_pmt, cmpr_data_size, part, -1);	/* Data not in cache */
	PMT_INDICATOR_SET_DIRTY(param->u4PMTIndicator[cluster]);
	mt_ftl_debug("u4PMTIndicator[%d] = 0x%x", cluster, param->u4PMTIndicator[cluster]);
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_UPDATE_PMT_MODIFYPMT);
	return ret;
}

/* Suppose FS_PAGE_SIZE for each write */
int mt_ftl_write(struct mt_ftl_blk *dev, char *buffer, sector_t sector, int len)
{
	int ret = MT_FTL_SUCCESS;
	int leb = 0, page = 0;
	u32 cluster = 0, sec_offset = 0;
	int cache_num = 0;
	int *meta_pmt = NULL;
	u32 cmpr_len = 0, data_offset = 0, total_consumed_size = 0;
	int last_data_len = 0;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;
	struct mt_ftl_param *param = dev->param;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_WRITEPAGE);
	MT_FTL_PROFILE_START_ALL(MT_FTL_PROFILE_WRITE_ALL);

	/* TODO: if the sector has been in cache, just modify it in cache, instead of write into nand */
#ifdef MT_FTL_SUPPORT_COMPR
	/* ret = crypto_comp_compress(param->cc, buffer, len, param->cmpr_page_buffer, &cmpr_len); */
	ret = mt_ftl_compress(dev, buffer, len, &cmpr_len);
	if (ret) {
		mt_ftl_err("ret = %d, cmpr_len = %d, len = 0x%x", ret, cmpr_len, len);
		mt_ftl_err("cc = 0x%lx, buffer = 0x%lx", (unsigned long int)param->cc, (unsigned long int)buffer);
		mt_ftl_err("cmpr_page_buffer = 0x%lx", (unsigned long int)param->cmpr_page_buffer);
		return MT_FTL_FAIL;
	}
#else
	param->cmpr_page_buffer = buffer;
	cmpr_len = len;
#endif

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_WRITEPAGE);

	if (MTKFTL_MAX_DATA_NUM_PER_PAGE < param->u4DataNum) {
		mt_ftl_err("MTKFTL_MAX_DATA_NUM_PER_PAGE(%d) < param->u4DataNum(%d)",
			   MTKFTL_MAX_DATA_NUM_PER_PAGE, param->u4DataNum);
	}

	if (param->u4DataNum > 0) {
		data_offset =
			((param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum].offset_len >> 16) & 0xFFFF)
			+ (param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum].offset_len & 0xFFFF);
		param->u4NextPageOffsetIndicator = 0;
	} else
		data_offset = param->u4NextPageOffsetIndicator;
	total_consumed_size = data_offset + cmpr_len + (param->u4DataNum + 1) * sizeof(struct mt_ftl_data_header) + 4;
	leb = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4NextLebPageIndicator);
	page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4NextLebPageIndicator);
	if ((total_consumed_size > dev->min_io_size)
		|| (param->u4DataNum >= MTKFTL_MAX_DATA_NUM_PER_PAGE)) {
		last_data_len = dev->min_io_size - data_offset -
			((param->u4DataNum + 1) * sizeof(struct mt_ftl_data_header) + 4);
		if ((page + 1) == page_num_per_block) {
			ret = mt_ftl_write_page(dev);
			data_offset = 0;
			last_data_len = 0;
		} else if (last_data_len <= 0) {
			mt_ftl_err("write_page=(%d:%d),data_offset=%d,last_data_len=%d",
				page, leb, data_offset, last_data_len);
			ret = mt_ftl_write_page(dev);
			last_data_len = 0;
			data_offset = 0;
		}
	}
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_WRITEPAGE);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_COPYTOCACHE);

	leb = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4NextLebPageIndicator);
	page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4NextLebPageIndicator);
	param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum - 1].sector =
	    (sector / (FS_PAGE_SIZE >> 9)) * (FS_PAGE_SIZE >> 9);
	param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - param->u4DataNum - 1].offset_len =
	    (data_offset << 16) | cmpr_len;
	if (last_data_len) {
		ubi_assert(data_offset + last_data_len < dev->min_io_size);
		memcpy(&param->u1DataCache[data_offset], param->cmpr_page_buffer, last_data_len);
	} else {
		ubi_assert(data_offset + cmpr_len < dev->min_io_size);
		memcpy(&param->u1DataCache[data_offset], param->cmpr_page_buffer, cmpr_len);
	}
	param->u4DataNum++;

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_COPYTOCACHE);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_UPDATEPMT);

	cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / dev->pm_per_io;
	sec_offset = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (dev->pm_per_io - 1);
	ret = mt_ftl_updatePMT(dev, cluster, sec_offset, leb, page * dev->min_io_size, param->u4DataNum - 1,
			 cmpr_len, false, true);
	if (ret < 0) {
		mt_ftl_err("mt_ftl_updatePMT cluster(%d) offset(%d) leb(%d) page(%d) fail\n",
			cluster, sec_offset, leb, page);
		return ret;
	}

	MT_FTL_PROFILE_END(MT_FTL_PROFILE_WRITE_UPDATEPMT);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_WRITEPAGE);
	ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
	cache_num = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[cluster]);
	ubi_assert(cache_num < PMT_CACHE_NUM);
	ubi_assert(sec_offset < dev->pm_per_io);
	meta_pmt = &param->u4MetaPMTCache[cache_num * dev->pm_per_io + sec_offset];
	if (param->u4DataNum != 0)
		PMT_SET_DATACACHE_BUF_NUM(*meta_pmt, 0);	/* Data is in cache 0 */

	PMT_INDICATOR_SET_DIRTY(param->u4PMTIndicator[cluster]);	/* Set corresponding PMT cache to dirty */
	if (last_data_len) {
		if (param->u4DataNum != 0)
			ret = mt_ftl_write_page(dev);
		data_offset = 0;
		param->u4NextPageOffsetIndicator = cmpr_len - last_data_len;
		memcpy(&param->u1DataCache[data_offset], &param->cmpr_page_buffer[last_data_len],
			param->u4NextPageOffsetIndicator);
		/*mt_ftl_err(dev, "u4NextPageOffsetIndicator = %u", param->u4NextPageOffsetIndicator);*/
		if ((dev->sync == 1) || (dev->flush == 1)) {
			ret = mt_ftl_commit(dev);
			if (ret)
				return ret;
			dev->flush = dev->sync = 0;
		}
	}
	if (dev->flush == 1) {
		mt_ftl_err("write flush");
		ret = mt_ftl_commit(dev);
		if (ret)
			return ret;
		dev->flush = dev->sync = 0;
	} else if (dev->sync == 1) {
		if (param->u4DataNum != 0) {
			ret = mt_ftl_write_page(dev);
			if (ret)
				return ret;
		}
		ret = ubi_sync(dev->ubi_num);
		dev->sync = 0;
	}
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_WRITE_WRITEPAGE);
	MT_FTL_PROFILE_END_ALL(MT_FTL_PROFILE_WRITE_ALL);
	return ret;
}

/* Suppose FS_PAGE_SIZE for each read */
int mt_ftl_read(struct mt_ftl_blk *dev, char *buffer, sector_t sector, int len)
{
	int ret = MT_FTL_SUCCESS;
	int leb = 0, page = 0, part = 0;
	u32 cluster = 0, sec_offset = 0;
	int pmt = 0, meta_pmt = 0;
	unsigned int offset_in_pagebuf = 0;
	int pmt_block = 0, pmt_page = 0;
	int cache_num = 0, data_cache_num = 0;
	u32 decmpr_len = 0;
	u32 data_num = 0, data_num_offset = 0;
	int next_block = 0, next_page = 0;
	int last_data_len = 0;
	struct mt_ftl_param *param = dev->param;
	u32 data_hdr_offset = dev->min_io_size - (param->u4DataNum * sizeof(struct mt_ftl_data_header) + 4);
	unsigned char *page_buffer = NULL;
	struct mt_ftl_data_header *header_buffer = NULL;

	MT_FTL_PROFILE_START(MT_FTL_PROFILE_READ_GETPMT);
	MT_FTL_PROFILE_START_ALL(MT_FTL_PROFILE_READ_ALL);

	cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / dev->pm_per_io;
	sec_offset = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (dev->pm_per_io - 1);

	ubi_assert(sec_offset <= dev->pm_per_io);
	ubi_assert(cluster <= PMT_TOTAL_CLUSTER_NUM);
	ret = mt_ftl_get_pmt(dev, sector, &pmt, &meta_pmt);
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_READ_GETPMT);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_READ_DATATOCACHE);
	/* Look up PMT in cache and get data from NAND flash */
	if (pmt == NAND_DEFAULT_VALUE) {
		/* mt_ftl_err(dev, "PMT of sector(0x%lx) is invalid", (unsigned long int)sector); */
		memset((void *)buffer, 0x0, len);
		return MT_FTL_SUCCESS;
	} else if (ret)
		return MT_FTL_FAIL;
	leb = PMT_GET_BLOCK(pmt);
	page = PMT_GET_PAGE(pmt);
	part = PMT_GET_PART(meta_pmt);

	/* mt_ftl_err(dev, "Copy to cache"); */
	ubi_assert(part < MTKFTL_MAX_DATA_NUM_PER_PAGE);
	if (PMT_IS_DATA_INCACHE(meta_pmt)) {
		mt_ftl_err("[INFO] Use data in cache");
		data_cache_num = PMT_GET_DATACACHENUM(meta_pmt);	/* Not used yet */
		header_buffer = &param->u4Header[MTKFTL_MAX_DATA_NUM_PER_PAGE - part - 1];
		offset_in_pagebuf = (header_buffer->offset_len >> 16) & 0xFFFF;
		if ((offset_in_pagebuf + (header_buffer->offset_len & 0xFFFF)) >= dev->min_io_size) {
			mt_ftl_err("(offset_in_pagebuf+(header_buffer->offset_len&0xFFFF))(%d)>=NAND_PAGE_SIZE(%d)"
				   , (offset_in_pagebuf + (header_buffer->offset_len & 0xFFFF)), dev->min_io_size);
			mt_ftl_err("offset_in_pagebuf = %d, (header_buffer->offset_len & 0xFFFF) = %d",
				   offset_in_pagebuf, (header_buffer->offset_len & 0xFFFF));
		}
		page_buffer = &param->u1DataCache[offset_in_pagebuf];
	} else {
		data_num_offset = (page + 1) * dev->min_io_size - 4;
		data_hdr_offset = data_num_offset - (part + 1) * sizeof(struct mt_ftl_data_header);

		ret = mt_ftl_leb_read(dev, leb, param->u4ReadHeader, data_hdr_offset,
			sizeof(struct mt_ftl_data_header));
		if (ret)
			return MT_FTL_FAIL;

		header_buffer = &param->u4ReadHeader[0];

		offset_in_pagebuf = page * dev->min_io_size + ((header_buffer->offset_len >> 16) & 0xFFFF);
		last_data_len = dev->min_io_size - ((header_buffer->offset_len >> 16) & 0xFFFF) -
			((part + 1) * sizeof(struct mt_ftl_data_header) + 4);
		if (last_data_len < (header_buffer->offset_len & 0xFFFF)) {
			next_block = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4NextLebPageIndicator);
			next_page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4NextLebPageIndicator);
			ret = mt_ftl_leb_read(dev, leb, param->general_page_buffer, offset_in_pagebuf, last_data_len);
			if (ret)
				return MT_FTL_FAIL;
			if ((leb == next_block) && ((page + 1) == next_page)) {
				memcpy((char *)param->general_page_buffer + last_data_len, param->u1DataCache,
					(header_buffer->offset_len & 0xFFFF) - last_data_len);
			} else {
				ret = mt_ftl_leb_read(dev, leb, (char *)param->general_page_buffer +
					last_data_len, (page + 1) * dev->min_io_size,
					(header_buffer->offset_len & 0xFFFF) - last_data_len);
				if (ret)
					return MT_FTL_FAIL;
			}
		} else {
			ret = mt_ftl_leb_read(dev, leb, param->general_page_buffer, offset_in_pagebuf,
					(header_buffer->offset_len & 0xFFFF));
			if (ret)
				return MT_FTL_FAIL;
		}
		page_buffer = (unsigned char *)param->general_page_buffer;
	}
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_READ_DATATOCACHE);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_READ_ADDRNOMATCH);

	/* mt_ftl_err(dev, "Check sector"); */
	if (header_buffer->sector != ((sector / (FS_PAGE_SIZE >> 9)) * (FS_PAGE_SIZE >> 9))) {
		if ((header_buffer->sector == 0xFFFFFFFF) && (page_buffer[0] == 0xFF)) {
			mt_ftl_err("sector(0x%lx) hasn't been written", (unsigned long int)sector);
			mt_ftl_err("leb = %d, page = %d, part = %d", leb, page, part);
			memset((void *)buffer, 0xFF, len);
			return MT_FTL_SUCCESS;
		}
		ret = mt_ftl_leb_read(dev, leb, param->tmp_page_buffer, page * dev->min_io_size, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		data_num = PAGE_GET_DATA_NUM(param->tmp_page_buffer[(dev->min_io_size >> 2) - 1]);
		mt_ftl_err("header_buffer[%d].sector(0x%lx) != sector (0x%lx), header_buffer[%d].offset_len = 0x%x",
			   part, (unsigned long int)header_buffer->sector,
			   (unsigned long int)sector, part, header_buffer->offset_len);
		mt_ftl_err("page_buffer[0] = 0x%x, u4PMTIndicator[%d] = 0x%x, data_num = %d",
			   page_buffer[0], cluster, param->u4PMTIndicator[cluster], data_num);
		mt_ftl_err("pmt = 0x%x, meta_pmt = 0x%x, leb = %d, page = %d, part = %d",
			   pmt, meta_pmt, leb, page, part);
		mt_ftl_err("data_num_offset = %d, data_hdr_offset = %d", data_num_offset,
			   data_hdr_offset);
		mt_ftl_err("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 1],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 2],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 3],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 4],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 5],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 6],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 7],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 8],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) - 9]);
		/*=====================Debug==========================*/
		/* Calculate clusters and sec_offsets */
		cluster =
		    ((unsigned long int)header_buffer->sector / (FS_PAGE_SIZE >> 9)) / dev->pm_per_io;
		sec_offset =
		    ((unsigned long int)header_buffer->sector /
		     (FS_PAGE_SIZE >> 9)) & (dev->pm_per_io - 1);
		mt_ftl_err("cluster = %d", cluster);
		mt_ftl_err("u4PMTIndicator[%d] = 0x%x", cluster,
			   param->u4PMTIndicator[cluster]);
		ubi_assert(sec_offset <= dev->pm_per_io);

		/* Download PMT to read PMT cache */
		/* Don't use mt_ftl_updatePMT, that will cause PMT indicator mixed in replay */
		if (PMT_INDICATOR_IS_INCACHE(param->u4PMTIndicator[cluster])) {
			cache_num = PMT_INDICATOR_CACHE_BUF_NUM(param->u4PMTIndicator[cluster]);
			ubi_assert(cache_num < PMT_CACHE_NUM);
			ubi_assert(sec_offset < dev->pm_per_io);
			pmt = param->u4PMTCache[cache_num * dev->pm_per_io + sec_offset];
			meta_pmt = param->u4MetaPMTCache[cache_num * dev->pm_per_io + sec_offset];
			/*mt_ftl_err( "[Debug] cluster is in write cache, cache_num = %d, pmt = 0x%x,
			  u4PMTIndicator[%d] = 0x%x",
			   cache_num, pmt, cluster, param->u4PMTIndicator[cluster]);    // Temporary */
		} else if (cluster == param->i4CurrentReadPMTClusterInCache) {
			/* mt_ftl_err( "[Debug] cluster == i4CurrentReadPMTClusterInCache (%d)",
			   param->i4CurrentReadPMTClusterInCache); */
			pmt = param->u4ReadPMTCache[sec_offset];
			meta_pmt = param->u4ReadMetaPMTCache[sec_offset];
		} else {
			pmt_block = PMT_INDICATOR_GET_BLOCK(param->u4PMTIndicator[cluster]);
			pmt_page = PMT_INDICATOR_GET_PAGE(param->u4PMTIndicator[cluster]);

			if (unlikely(pmt_block == 0)) {
				mt_ftl_err("pmt_block == 0");
				/* memset(param->u4ReadPMTCache, 0xFF, PM_PER_NANDPAGE * sizeof(unsigned int)); */
				pmt = 0xFFFFFFFF;
				meta_pmt = 0xFFFFFFFF;
			} else {
				mt_ftl_err("Get PMT of cluster (%d)", cluster);
				ret = mt_ftl_leb_read(dev, pmt_block, param->u4ReadPMTCache,
						pmt_page * dev->min_io_size, dev->min_io_size);
				if (ret)
					return MT_FTL_FAIL;
				ret = mt_ftl_leb_read(dev, pmt_block, param->u4ReadMetaPMTCache,
					(pmt_page + 1) * dev->min_io_size, dev->min_io_size);
				if (ret)
					return MT_FTL_FAIL;
				param->i4CurrentReadPMTClusterInCache = cluster;
				pmt = param->u4ReadPMTCache[sec_offset];
				meta_pmt = param->u4ReadMetaPMTCache[sec_offset];
			}
		}

		leb = PMT_GET_BLOCK(pmt);
		page = PMT_GET_PAGE(pmt);
		part = PMT_GET_PART(meta_pmt);
		mt_ftl_err("for sector (0x%lx), pmt = 0x%x, meta_pmt = 0x%x, leb = %d, page = %d, part = %d",
			   (unsigned long int)header_buffer->sector, pmt, meta_pmt, leb, page, part);
		/*====================================================*/
		return MT_FTL_FAIL;
	}
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_READ_ADDRNOMATCH);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_READ_DECOMP);

	decmpr_len = dev->min_io_size;
#ifdef MT_FTL_SUPPORT_COMPR
	/* ret =
	    crypto_comp_decompress(param->cc, page_buffer, (header_buffer->offset_len & 0xFFFF),
				   param->cmpr_page_buffer, &decmpr_len); */
	ret = mt_ftl_decompress(dev, page_buffer, (header_buffer->offset_len & 0xFFFF), &decmpr_len);
	if (ret) {
		ret = mt_ftl_leb_read(dev, leb, param->tmp_page_buffer, page * dev->min_io_size, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		mt_ftl_err("part = %d", part);
		mt_ftl_err("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1)],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 1],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 2],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 3],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 4],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 5],
			   param->tmp_page_buffer[(dev->min_io_size >> 2) -
						  ((part + 2) * sizeof(struct mt_ftl_data_header) + 1) + 6]);
		mt_ftl_err("ret = %d, decmpr_len = %d, header_buffer->offset_len = 0x%x",
			   ret, decmpr_len, header_buffer->offset_len);
		mt_ftl_err("cc = 0x%lx, page_buffer = 0x%lx",
			   (unsigned long int)param->cc, (unsigned long int)page_buffer);
		mt_ftl_err("cmpr_page_buffer = 0x%lx", (unsigned long int)param->cmpr_page_buffer);
		return MT_FTL_FAIL;
	}
#endif
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_READ_DECOMP);
	MT_FTL_PROFILE_START(MT_FTL_PROFILE_READ_COPYTOBUFF);

	offset_in_pagebuf = ((sector % (FS_PAGE_SIZE >> 9)) << 9);
	if ((offset_in_pagebuf + len) > decmpr_len) {
		mt_ftl_err("offset(%d)+len(%d)>decmpr_len (%d)", offset_in_pagebuf, len, decmpr_len);
		return MT_FTL_FAIL;
	}
	/* mt_ftl_err( "Copy to buffer"); */
	ubi_assert(offset_in_pagebuf + len <= dev->min_io_size);
#ifdef MT_FTL_SUPPORT_COMPR
	memcpy((void *)buffer, &param->cmpr_page_buffer[offset_in_pagebuf], len);
#else
	memcpy((void *)buffer, (page_buffer + offset_in_pagebuf), len);
#endif
	MT_FTL_PROFILE_END(MT_FTL_PROFILE_READ_COPYTOBUFF);
	MT_FTL_PROFILE_END_ALL(MT_FTL_PROFILE_READ_ALL);
	return ret;
}

static int mt_ftl_replay_single_block(struct mt_ftl_blk *dev, int leb, int *page)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int offset = 0;
	sector_t sector = 0;
	u32 cluster = 0, sec_offset = 0;
	u32 data_num = 0, data_hdr_offset = 0;
	int retry = 0, last_data_len = 0;
	struct mt_ftl_data_header *header_buffer = NULL;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;

	ret = ubi_is_mapped(desc, leb);
	if (ret == 0) {
		mt_ftl_err("leb%d is unmapped", leb);
		return MT_FTL_FAIL;
	}
	/* Check if the block is PMT block
	 * If yes, check the correctness of param->u4CurrentPMTLebPageIndicator
	 */
	offset = (*page) * dev->min_io_size;
	ret = mt_ftl_leb_read(dev, leb, param->general_page_buffer, offset, dev->min_io_size);
	if (ret)
		return MT_FTL_FAIL;

	data_num = PAGE_GET_DATA_NUM(param->general_page_buffer[(dev->min_io_size >> 2) - 1]);
	if (data_num == 0x7FFFFFFF) {
		mt_ftl_err("End of block");
		return MT_FTL_SUCCESS;
	}
	data_hdr_offset = dev->min_io_size - (data_num * sizeof(struct mt_ftl_data_header) + 4);
	if (dev->min_io_size <= data_hdr_offset) {
		mt_ftl_err("NAND_PAGE_SIZE(%d) <= data_hdr_offset(%d) data_num(%d)",
			   dev->min_io_size, data_hdr_offset, data_num);
	}
	header_buffer = (struct mt_ftl_data_header *)(&param->general_page_buffer[data_hdr_offset >> 2]);

	mt_ftl_err("leb = %d, page = %d, data_num = %d", leb, *page, data_num);
	while (data_num && (offset <= max_offset_per_block)) {
		/* Update param->u4NextLebPageIndicator
		 * check the correctness of param->u4NextLebPageIndicator &
		 * if param->u4NextLebPageIndicator is full, need to call get free block & page function
		 */

		/* Update param->u4PMTCache and param->u4PMTIndicator and param->u4BIT */
		/* If the page is copied in GC, that means the page should not be replayed */
		retry = 1;
		last_data_len = (((header_buffer[0].offset_len >> 16) & 0xFFFF) +
			(header_buffer[0].offset_len & 0xFFFF) + (data_num * sizeof(struct mt_ftl_data_header) + 4))
			> dev->min_io_size;
		if (PAGE_BEEN_READ(param->general_page_buffer[(dev->min_io_size >> 2) - 1]) == 0) {
			for (i = 0; i < data_num; i++) {
				/* Get sector in the page */
				sector = header_buffer[data_num - i - 1].sector;
				if ((sector & NAND_DEFAULT_SECTOR_VALUE) == NAND_DEFAULT_SECTOR_VALUE) {
					mt_ftl_err("header_buffer[%d].sector == 0xFFFFFFFF, leb = %d, page = %d",
						   i, leb, offset / dev->min_io_size);
					continue;
				}

				/* Calculate clusters and sec_offsets */
				cluster = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) / dev->pm_per_io;
				sec_offset = ((unsigned long int)sector / (FS_PAGE_SIZE >> 9)) & (dev->pm_per_io - 1);

				mt_ftl_updatePMT(dev, cluster, sec_offset, leb, offset, i,
						 (header_buffer[data_num - i - 1].offset_len &
						  0xFFFF), true, true);
			}
		}
retry_once:
		offset += dev->min_io_size;
		*page = offset / dev->min_io_size;
		if (offset > max_offset_per_block)
			break;
		PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, leb, *page);
		ret = mt_ftl_leb_read(dev, leb, param->general_page_buffer, offset, dev->min_io_size);
		if (ret == MT_FTL_FAIL) {
			/* write page power loss recover */
			ret = mt_ftl_leb_recover(dev, leb, offset, 0);
			if (ret == 1)
				ret = MT_FTL_SUCCESS;
			break;
		}

		data_num = PAGE_GET_DATA_NUM(param->general_page_buffer[(dev->min_io_size >> 2) - 1]);
		mt_ftl_err("%d, %d, %u, %d", retry, last_data_len, data_num, *page);
		if (data_num == 0x7FFFFFFF) {
			if (retry && last_data_len) {
				retry = 0;
				goto retry_once;
			}
			break;
		}
		data_hdr_offset = dev->min_io_size - (data_num * sizeof(struct mt_ftl_data_header) + 4);
		if (dev->min_io_size <= data_hdr_offset) {
			mt_ftl_err("NAND_PAGE_SIZE(%d) <= data_hdr_offset(%d) data_num=0x%x",
				   dev->min_io_size, data_hdr_offset, data_num);
		}
		header_buffer = (struct mt_ftl_data_header *)(&param->general_page_buffer[data_hdr_offset >> 2]);
	}

	mt_ftl_err("offset = %d at the end, max_offset_per_block = %d", offset, max_offset_per_block);

	return ret;
}

static int mt_ftl_check_single_blk(struct mt_ftl_blk *dev, int offset, int *leb)
{
	int ret1 = 0, ret2 = 0;
	int leb1 = 0, leb2 = 0;
	int  magic1 = 0, magic2 = 0;
	struct mt_ftl_param *param = dev->param;

	*leb = MT_INVALID_BLOCKPAGE;
	param->u4NextReplayOffsetIndicator = offset;
	ret1 = mt_ftl_leb_read(dev, REPLAY_BLOCK, param->replay_page_buffer, offset, sizeof(unsigned int) * 2);
	magic1 = param->replay_page_buffer[0];
	leb1 = param->replay_page_buffer[1];
	mt_ftl_err("[Bean1]%d %d", leb1, offset);
	offset += dev->min_io_size;
	ret2 = mt_ftl_leb_read(dev, REPLAY_BLOCK, param->replay_page_buffer, offset, sizeof(unsigned int) * 2);
	magic2 = param->replay_page_buffer[0];
	leb2 = param->replay_page_buffer[1];
	mt_ftl_err("[Bean2]%d %d", leb2, offset);
	if (magic1 == MT_MAGIC_NUMBER) {
		*leb = leb1;
		if (magic2 != MT_MAGIC_NUMBER || leb1 != leb2) {
			mt_ftl_err("magic(0x%x) or leb1(%d) != leb2(%d)", magic2, leb1, leb2);
			return 2; /* case 2 for gc power cut  */
		}
		return 1; /* case 1 for gc done */
	} else if (magic2 == MT_MAGIC_NUMBER) {
		*leb = leb2;
		offset += dev->min_io_size;
		ret1 = mt_ftl_leb_read(dev, REPLAY_BLOCK, param->replay_page_buffer, offset,
			sizeof(unsigned int) * 2);
		magic1 = param->replay_page_buffer[0];
		leb1 = param->replay_page_buffer[1];
		mt_ftl_err("[Bean3]%d %d", leb1, offset);
		if (leb1 == leb2) {
			mt_ftl_err("[Bean]u4NextReplayOffsetIndicator write error, fix me");
			ubi_assert(false);
			return 1; /* error case 1 for gc done , fix me*/
		}
	}
	return 0; /* case 0 for no replay */
}

static int mt_ftl_check_reserved_blk(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS;
	struct mt_ftl_param *param = dev->param;

	ret = mt_ftl_leb_map(dev, param->u4GCReserveLeb);
	if (ret > 0) {
		/* check gc reserved block empty or not */
		ret = mt_ftl_leb_read(dev, param->u4GCReserveLeb, param->replay_page_buffer, 0, dev->min_io_size);
		if (param->replay_page_buffer[(dev->min_io_size >> 2) - 1] != 0xFFFFFFFF) {
			mt_ftl_err("GCReserveLeb(%d) not to remap, need to do", param->u4GCReserveLeb);
			mt_ftl_leb_remap(dev, param->u4GCReserveLeb);
		}
	}
	ret = mt_ftl_leb_map(dev, param->u4GCReserveLeb);
	if (ret > 0) {
		ret = mt_ftl_leb_read(dev, param->u4GCReservePMTLeb, param->replay_page_buffer, 0, dev->min_io_size);
		if (param->replay_page_buffer[(dev->min_io_size >> 2) - 1] != 0xFFFFFFFF) {
			mt_ftl_err("GCPMTReserveLeb(%d) not to remap, need to do", param->u4GCReservePMTLeb);
			mt_ftl_leb_remap(dev, param->u4GCReservePMTLeb);
		}
	}
	return ret;
}

static int mt_ftl_replay(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS, check_num = 0;
	int leb = 0, page = 0, offset = 0, full = 1, first = 0;
	int nextleb_in_replay = MT_INVALID_BLOCKPAGE, tmp_leb;
	int pmt_block = 0, pmt_page = 0;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	param->replay_blk_index = 0;
	/* check the next or not */
	check_num = mt_ftl_check_single_blk(dev, offset, &tmp_leb);
	if (check_num != 1) {
		mt_ftl_err("first last replay start");
		MT_FTL_PARAM_LOCK(dev);
		last_replay_flag = REPLAY_LAST_BLK;
		MT_FTL_PARAM_UNLOCK(dev);
	}
	/* Replay leb/page of param->u4NextLebPageIndicator */
	leb = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4NextLebPageIndicator);
	page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4NextLebPageIndicator);
	ret = mt_ftl_replay_single_block(dev, leb, &page);
	if (ret)
		return ret;
	if (page == page_num_per_block) {
		mt_ftl_err("replay getfreeblock");
		first = 1;
	} else {
		mt_ftl_err("replay nextlebpage only");
		PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, leb, page);
		last_replay_flag = REPLAY_END;
		goto out_recover;
	}
	/* Get the successive lebs to replay */
	ret = ubi_is_mapped(desc, REPLAY_BLOCK);
	if (ret == 0) {
		mt_ftl_err("leb%d is unmapped", REPLAY_BLOCK);
		return MT_FTL_FAIL;
	}
retry:
	while (offset < max_offset_per_block) {
		ret = mt_ftl_check_single_blk(dev, offset, &nextleb_in_replay);
		switch (ret) {
		case 0:
			mt_ftl_err("[Bean]no replay blk");
			if (page == page_num_per_block)
				last_replay_flag = REPLAY_END;
			goto out_recover;
		case 1:
			/* check the next or not */
			check_num = mt_ftl_check_single_blk(dev, offset + (dev->min_io_size << 1), &tmp_leb);
			if (check_num != 1) {
				mt_ftl_err("last replay start");
				MT_FTL_PARAM_LOCK(dev);
				last_replay_flag = REPLAY_LAST_BLK;
				MT_FTL_PARAM_UNLOCK(dev);
			}
			/* check the first get free block */
			if (full) {
				if (first) {
					/* get the first free leb */
					leb = mt_ftl_getfreeblock(dev, &page, false, true);
					first = 0;
				}
				mt_ftl_err("[Bean]nextlebpage replay done %d %d", leb, nextleb_in_replay);
				if (leb == nextleb_in_replay) {
					mt_ftl_err("replay next leb in replay block");
					full = 0;
					break;
				} else {
					offset += (dev->min_io_size << 1);
					mt_ftl_err("replay replayblock mapped");
					goto retry;
				}
			}
			leb = mt_ftl_getfreeblock(dev, &page, false, true);
			if (leb != nextleb_in_replay) {
				mt_ftl_err("leb(%d)!=nextleb_in_replay(%d)", leb, nextleb_in_replay);
				return MT_FTL_FAIL;
			}
			break;
		case 2:
			mt_ftl_err("[Bean]replay nextleb_in_replay block");
			mt_ftl_leb_remap(dev, nextleb_in_replay);
			leb = mt_ftl_getfreeblock(dev, &page, false, false);
			if (leb != nextleb_in_replay) {
				mt_ftl_err("leb(%d)!=nextleb_in_replay(%d)", leb, nextleb_in_replay);
				ret = MT_FTL_FAIL;
			} else {
				PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, leb, page);
				ret = MT_FTL_SUCCESS;
			}
			offset = 0;
			goto out;
		default:
			mt_ftl_err("[Bean]mt_ftl_check_single_blk fail");
			return MT_FTL_FAIL;
		}
		offset += (dev->min_io_size << 1);
		page = 0;
		ret = mt_ftl_replay_single_block(dev, leb, &page);
		if (ret)
			return ret;
	}
out_recover:
	/* recover power cut data block */
	ret = mt_ftl_leb_recover(dev, leb, page * dev->min_io_size, 0);
	if (ret == MT_FTL_FAIL) {
		mt_ftl_err("recover data blk fail %d %d", leb, page);
		return ret;
	}
	/* recover power cut pmt block */
	if (last_replay_flag == REPLAY_END) {
		pmt_block = PMT_LEB_PAGE_INDICATOR_GET_BLOCK(param->u4CurrentPMTLebPageIndicator);
		pmt_page = PMT_LEB_PAGE_INDICATOR_GET_PAGE(param->u4CurrentPMTLebPageIndicator);
		ret = mt_ftl_leb_recover(dev, pmt_block, pmt_page * dev->min_io_size, 0);
		if (ret == MT_FTL_FAIL) {
			mt_ftl_err("recover pmt blk fail %d %d", pmt_block, pmt_page);
			return ret;
		}
	}
	if (page == page_num_per_block) {
		leb = mt_ftl_getfreeblock(dev, &page, false, false);
		PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, leb, page);
	}
	ret = MT_FTL_SUCCESS;
out:
	ret = mt_ftl_check_reserved_blk(dev);
	/* replay over force commit by Bean*/
	mt_ftl_commit(dev);
	MT_FTL_PARAM_LOCK(dev);
	recorded_pmt_blk = -1; /* reset  recorded pmt blk */
	last_replay_flag = REPLAY_EMPTY;
	MT_FTL_PARAM_UNLOCK(dev);
	return ret;
}

static int mt_ftl_alloc_single_buffer(unsigned int **buf, int size, char *str)
{
	if (*buf == NULL) {
		*buf = kzalloc(size, GFP_KERNEL);
		if (!*buf) {
			mt_ftl_err("%s allocate memory fail", str);
			return -ENOMEM;
		}
	}
	memset(*buf, 0xFF, size);

	return 0;
}

static int mt_ftl_alloc_buffers(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	ret = mt_ftl_alloc_single_buffer(&param->u4PMTIndicator,
					 PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int),
					 "param->u4PMTIndicator");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->u4PMTCache,
					 dev->pm_per_io * PMT_CACHE_NUM * sizeof(unsigned int),
					 "param->u4PMTCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->u4MetaPMTCache,
					 dev->pm_per_io * PMT_CACHE_NUM * sizeof(unsigned int),
					 "param->u4MetaPMTCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer((unsigned int **)&param->i4CurrentPMTClusterInCache,
					 PMT_CACHE_NUM * sizeof(unsigned int),
					 "param->i4CurrentPMTClusterInCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->u4ReadPMTCache,
					 dev->pm_per_io * sizeof(unsigned int),
					 "param->u4ReadPMTCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->u4ReadMetaPMTCache,
					 dev->pm_per_io * sizeof(unsigned int),
					 "param->u4ReadMetaPMTCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->u4BIT,
					 NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int),
					 "param->u4BIT");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer((unsigned int **)&param->u1DataCache,
					 dev->min_io_size * sizeof(unsigned char),
					 "param->u1DataCache");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer((unsigned int **)&param->u4Header,
					 MTKFTL_MAX_DATA_NUM_PER_PAGE *
					 sizeof(struct mt_ftl_data_header), "param->u4Header");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer((unsigned int **)&param->u4ReadHeader,
					 MTKFTL_MAX_DATA_NUM_PER_PAGE *
					 sizeof(struct mt_ftl_data_header), "param->u4ReadHeader");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->replay_blk_rec,
					 page_num_per_block * sizeof(unsigned int),
					 "param->replay_blk_rec");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->general_page_buffer,
					 (dev->min_io_size >> 2) * sizeof(unsigned int),
					 "param->general_page_buffer");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->replay_page_buffer,
					 (dev->min_io_size >> 2) * sizeof(unsigned int),
					 "param->replay_page_buffer");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->commit_page_buffer,
					 (dev->min_io_size >> 2) * sizeof(unsigned int),
					 "param->commit_page_buffer");
	if (ret)
		return ret;

	ret = mt_ftl_alloc_single_buffer(&param->gc_page_buffer,
					 (dev->min_io_size >> 2) * sizeof(unsigned int),
					 "param->gc_page_buffer");
	if (ret)
		return ret;
#ifdef	MT_FTL_SUPPORT_COMPR
	ret = mt_ftl_alloc_single_buffer((unsigned int **)&param->cmpr_page_buffer,
					 dev->min_io_size * sizeof(unsigned char),
					 "param->cmpr_page_buffer");
	if (ret)
		return ret;
#endif
	ret = mt_ftl_alloc_single_buffer(&param->tmp_page_buffer,
					 (dev->min_io_size >> 2) * sizeof(unsigned int),
					 "param->tmp_page_buffer");
	if (ret)
		return ret;

	return ret;
}

static int mt_ftl_check_img_reload(struct mt_ftl_blk *dev, int leb)
{
	int ret = MT_FTL_SUCCESS;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;
	int offset = 0;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;

	ret = ubi_is_mapped(desc, leb);
	if (ret == 0) {
		mt_ftl_err("leb%d is unmapped", leb);
		return MT_FTL_FAIL;
	}
	while (offset <= max_offset_per_block) {
		ret = mt_ftl_leb_read(dev, leb, param->tmp_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		if (param->tmp_page_buffer[0] == 0x00000000) {
			mt_ftl_err("image reloaded, offset = %d", offset);
			return 1;
		}
		offset += dev->min_io_size;
	}

	mt_ftl_debug("image not reloaded offset = %d", offset);
	return 0;
}

static int mt_ftl_recover_blk(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	int offset = 0;
	struct mt_ftl_param *param = dev->param;
	const int max_offset_per_block = dev->leb_size - dev->min_io_size;
	/* Recover Config Block */
	ret = mt_ftl_leb_read(dev, CONFIG_START_BLOCK, param->general_page_buffer, offset, dev->min_io_size);
	if (ret)
		return MT_FTL_FAIL;
	mt_ftl_err("param->general_page_buffer[0] = 0x%x, MT_MAGIC_NUMBER = 0x%x",
		param->general_page_buffer[0], MT_MAGIC_NUMBER);
	mt_ftl_leb_remap(dev, CONFIG_START_BLOCK);
	mt_ftl_write_to_blk(dev, CONFIG_START_BLOCK, param->general_page_buffer);

	/* Recover Backup Config Block */
	mt_ftl_leb_remap(dev, CONFIG_START_BLOCK + 1);

	/* Recover Replay Blocks */
	mt_ftl_leb_remap(dev, REPLAY_BLOCK);

	/* Recover PMT Blocks */
	for (i = PMT_START_BLOCK + 1; i < PMT_START_BLOCK + PMT_BLOCK_NUM; i++)
		mt_ftl_leb_remap(dev, i);
	while (offset <= max_offset_per_block) {
		ret = mt_ftl_leb_read(dev, PMT_START_BLOCK, param->general_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		if (param->general_page_buffer[0] == 0x00000000) {
			mt_ftl_err("offset = %d, page = %d", offset, offset / dev->min_io_size);
			break;
		}
		ret = mt_ftl_leb_write(dev, PMT_START_BLOCK + 1, param->general_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		offset += dev->min_io_size;
	}
	offset = 0;
	mt_ftl_leb_remap(dev, PMT_START_BLOCK);
	while (offset <= max_offset_per_block) {
		ret = mt_ftl_leb_read(dev, PMT_START_BLOCK + 1, param->general_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		if (param->general_page_buffer[0] == 0xFFFFFFFF) {
			mt_ftl_err("offset = %d, page = %d", offset, offset / dev->min_io_size);
			break;
		}
		ret = mt_ftl_leb_write(dev, PMT_START_BLOCK, param->general_page_buffer, offset, dev->min_io_size);
		if (ret)
			return MT_FTL_FAIL;
		offset += dev->min_io_size;
	}
	mt_ftl_leb_remap(dev, PMT_START_BLOCK + 1);

	return ret;
}

static int mt_ftl_show_param(struct mt_ftl_blk *dev)
{
	struct mt_ftl_param *param = dev->param;

	mt_ftl_err("u4NextPageOffsetIndicator = 0x%x", param->u4NextPageOffsetIndicator);
	mt_ftl_err("u4NextReplayOffsetIndicator = 0x%x", param->u4NextReplayOffsetIndicator);
	mt_ftl_err("u4NextLebPageIndicator = 0x%x", param->u4NextLebPageIndicator);
	mt_ftl_err("u4CurrentPMTLebPageIndicator = 0x%x", param->u4CurrentPMTLebPageIndicator);
	mt_ftl_err("u4NextFreeLebIndicator = 0x%x", param->u4NextFreeLebIndicator);
	mt_ftl_err("u4NextFreePMTLebIndicator = 0x%x", param->u4NextFreePMTLebIndicator);
	mt_ftl_err("u4GCReserveLeb = 0x%x", param->u4GCReserveLeb);
	mt_ftl_err("u4GCReservePMTLeb = 0x%x", param->u4GCReservePMTLeb);
	mt_ftl_err("u4PMTIndicator = 0x%x, 0x%x, 0x%x, 0x%x",
		   param->u4PMTIndicator[0],
		   param->u4PMTIndicator[1], param->u4PMTIndicator[2], param->u4PMTIndicator[3]);
	mt_ftl_err("u4BIT = 0x%x, 0x%x, 0x%x, 0x%x",
		   param->u4BIT[0], param->u4BIT[1], param->u4BIT[2], param->u4BIT[3]);
	mt_ftl_err("i4CurrentPMTClusterInCache = 0x%x, 0x%x, 0x%x, 0x%x",
		   param->i4CurrentPMTClusterInCache[0],
		   param->i4CurrentPMTClusterInCache[1],
		   param->i4CurrentPMTClusterInCache[2], param->i4CurrentPMTClusterInCache[3]);

	return MT_FTL_SUCCESS;
}

int mt_ftl_discard(struct mt_ftl_blk *dev, unsigned long sector, unsigned nr_sects)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;
	const int page_num_per_block = dev->leb_size / dev->min_io_size;

	param->u4DataNum = 0;
	param->replay_blk_index = 0;
	param->i4CurrentReadPMTClusterInCache = 0xFFFFFFFF;
	param->u4NextReplayOffsetIndicator = 0;
	param->u4NextPageOffsetIndicator = 0;

	/* There are some download information stored in some blocks
	   So unmap the blocks at first */
	for (i = 0; i < dev->dev_blocks; i++)
		ubi_leb_unmap(desc, i);

	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK);
	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK + 1);
	ret = mt_ftl_leb_map(dev, REPLAY_BLOCK);

	PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, DATA_START_BLOCK, 0);
	mt_ftl_err("u4NextLebPageIndicator = 0x%x", param->u4NextLebPageIndicator);
	ret = mt_ftl_leb_map(dev, DATA_START_BLOCK);

	PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4CurrentPMTLebPageIndicator, PMT_START_BLOCK, 0);
	mt_ftl_err("u4CurrentPMTLebPageIndicator = 0x%x", param->u4CurrentPMTLebPageIndicator);
	ret = mt_ftl_leb_map(dev, PMT_START_BLOCK);

	param->u4NextFreeLebIndicator = DATA_START_BLOCK + 1;
	mt_ftl_err("u4NextFreeLebIndicator = 0x%x", param->u4NextFreeLebIndicator);
	param->u4NextFreePMTLebIndicator = PMT_START_BLOCK + 1;
	mt_ftl_err("u4NextFreePMTLebIndicator = %d", param->u4NextFreePMTLebIndicator);
	param->u4GCReserveLeb = dev->dev_blocks - 1;
	mt_ftl_err("u4GCReserveLeb = %d", param->u4GCReserveLeb);
	ret = mt_ftl_leb_map(dev, param->u4GCReserveLeb);
	param->u4GCReservePMTLeb = DATA_START_BLOCK - 1;
	ret = mt_ftl_leb_map(dev, param->u4GCReservePMTLeb);

	memset(param->u4PMTIndicator, 0, PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int));
	memset(param->u4BIT, 0, NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int));
	/* add memory reset */
	memset(param->u4PMTCache, 0xFF, dev->pm_per_io * PMT_CACHE_NUM * sizeof(unsigned int));
	memset(param->u4MetaPMTCache, 0xFF, dev->pm_per_io * PMT_CACHE_NUM * sizeof(unsigned int));
	memset(param->i4CurrentPMTClusterInCache, 0xFF, PMT_CACHE_NUM * sizeof(unsigned int));
	memset(param->u4ReadPMTCache, 0xFF, dev->pm_per_io * sizeof(unsigned int));
	memset(param->u4ReadMetaPMTCache, 0xFF, dev->pm_per_io * sizeof(unsigned int));
	memset(param->u1DataCache, 0xFF, dev->min_io_size * sizeof(unsigned char));
	memset(param->u4Header, 0xFF, MTKFTL_MAX_DATA_NUM_PER_PAGE*sizeof(struct mt_ftl_data_header));
	memset(param->u4ReadHeader, 0xFF, MTKFTL_MAX_DATA_NUM_PER_PAGE*sizeof(struct mt_ftl_data_header));
	memset(param->replay_blk_rec, 0xFF, page_num_per_block * sizeof(unsigned int));
	memset(param->general_page_buffer, 0xFF, (dev->min_io_size >> 2) * sizeof(unsigned int));
	memset(param->replay_page_buffer, 0xFF, (dev->min_io_size >> 2) * sizeof(unsigned int));
	memset(param->commit_page_buffer, 0xFF, (dev->min_io_size >> 2) * sizeof(unsigned int));
	memset(param->gc_page_buffer, 0xFF, (dev->min_io_size >> 2) * sizeof(unsigned int));
#ifdef MT_FTL_SUPPORT_COMPR
	memset(param->cmpr_page_buffer, 0xFF, dev->min_io_size * sizeof(unsigned char));
#endif
	memset(param->tmp_page_buffer, 0xFF, (dev->min_io_size >> 2) * sizeof(unsigned int));

	mt_ftl_show_param(dev);
	return 0;
}

static int mt_ftl_param_default(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS, i = 0;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;

	param->u4DataNum = 0;
	param->replay_blk_index = 0;
	param->i4CurrentReadPMTClusterInCache = 0xFFFFFFFF;
#ifdef MT_FTL_SUPPORT_COMPR
	param->cc = crypto_alloc_comp("lzo", 0, 0);
#else
	param->cc = NULL;
#endif
	param->u4NextReplayOffsetIndicator = 0;
	param->u4NextPageOffsetIndicator = 0;

	/* There are some download information stored in some blocks
	   So unmap the blocks at first */
	for (i = 0; i < dev->dev_blocks; i++)
		ubi_leb_unmap(desc, i);

	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK);
	ret = mt_ftl_leb_map(dev, CONFIG_START_BLOCK + 1);
	ret = mt_ftl_leb_map(dev, REPLAY_BLOCK);

	PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4NextLebPageIndicator, DATA_START_BLOCK, 0);
	mt_ftl_err("u4NextLebPageIndicator = 0x%x", param->u4NextLebPageIndicator);
	ret = mt_ftl_leb_map(dev, DATA_START_BLOCK);

	PMT_LEB_PAGE_INDICATOR_SET_BLOCKPAGE(param->u4CurrentPMTLebPageIndicator, PMT_START_BLOCK, 0);
	mt_ftl_err("u4CurrentPMTLebPageIndicator = 0x%x", param->u4CurrentPMTLebPageIndicator);
	ret = mt_ftl_leb_map(dev, PMT_START_BLOCK);

	param->u4NextFreeLebIndicator = DATA_START_BLOCK + 1;
	mt_ftl_err("u4NextFreeLebIndicator = 0x%x", param->u4NextFreeLebIndicator);
	param->u4NextFreePMTLebIndicator = PMT_START_BLOCK + 1;
	mt_ftl_err("u4NextFreePMTLebIndicator = %d", param->u4NextFreePMTLebIndicator);
	param->u4GCReserveLeb = dev->dev_blocks - 1;
	mt_ftl_err("u4GCReserveLeb = %d", param->u4GCReserveLeb);
	ret = mt_ftl_leb_map(dev, param->u4GCReserveLeb);
	param->u4GCReservePMTLeb = DATA_START_BLOCK - 1;
	ret = mt_ftl_leb_map(dev, param->u4GCReservePMTLeb);

	memset(param->u4PMTIndicator, 0, PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int));
	memset(param->u4BIT, 0, NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int));

	mt_ftl_show_param(dev);

	/* Replay */
	ret = mt_ftl_replay(dev);
	if (ret) {
		mt_ftl_err("mt_ftl_replay fail, ret = %d", ret);
		return ret;
	}

	mt_ftl_show_param(dev);

	/* Commit indicators */
	mt_ftl_commit_indicators(dev);
	if (ret) {
		mt_ftl_err("mt_ftl_commit_indicators fail, ret = %d", ret);
		return ret;
	}

	return ret;
}

static int mt_ftl_param_init(struct mt_ftl_blk *dev, u32 *buffer)
{
	int ret = MT_FTL_SUCCESS;
	int index = 0;
	struct mt_ftl_param *param = dev->param;

	param->u4DataNum = 0;
	param->replay_blk_index = 0;
	param->i4CurrentReadPMTClusterInCache = 0xFFFFFFFF;
#ifdef MT_FTL_SUPPORT_COMPR
	param->cc = crypto_alloc_comp("lzo", 0, 0);
#else
	param->cc = NULL;
#endif
	param->u4NextPageOffsetIndicator = 0;
	param->u4NextReplayOffsetIndicator = buffer[1];
	param->u4NextLebPageIndicator = buffer[2];
	param->u4CurrentPMTLebPageIndicator = buffer[3];
	param->u4NextFreeLebIndicator = buffer[4];
	param->u4NextFreePMTLebIndicator = buffer[5];
	param->u4GCReserveLeb = buffer[6];
	if (param->u4GCReserveLeb == NAND_DEFAULT_VALUE)
		param->u4GCReserveLeb = dev->dev_blocks - 1;
	param->u4GCReservePMTLeb = buffer[7];
	if (param->u4GCReservePMTLeb == NAND_DEFAULT_VALUE)
		param->u4GCReservePMTLeb = DATA_START_BLOCK - 1;

	index = 8;
	if ((index + PMT_TOTAL_CLUSTER_NUM) * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("(index + PMT_TOTAL_CLUSTER_NUM) * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   (index + PMT_TOTAL_CLUSTER_NUM) * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, PMT_TOTAL_CLUSTER_NUM = %d", index, PMT_TOTAL_CLUSTER_NUM);
		return MT_FTL_FAIL;
	}
	memcpy(param->u4PMTIndicator, &buffer[index], PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int));

	index += ((PMT_TOTAL_CLUSTER_NUM * sizeof(unsigned int)) >> 2);
	if ((index + NAND_TOTAL_BLOCK_NUM) * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("(index + NAND_TOTAL_BLOCK_NUM) * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   (index + NAND_TOTAL_BLOCK_NUM) * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, NAND_TOTAL_BLOCK_NUM = %d", index, NAND_TOTAL_BLOCK_NUM);
		return MT_FTL_FAIL;
	}
	memcpy(param->u4BIT, &buffer[index], NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int));

	index += ((NAND_TOTAL_BLOCK_NUM * sizeof(unsigned int)) >> 2);
	if ((index + PMT_CACHE_NUM) * sizeof(unsigned int) > dev->min_io_size) {
		mt_ftl_err("(index + PMT_CACHE_NUM) * sizeof(unsigned int)(0x%lx) > NAND_PAGE_SIZE(%d)",
			   (index + PMT_CACHE_NUM) * sizeof(unsigned int), dev->min_io_size);
		mt_ftl_err("index = %d, PMT_CACHE_NUM = %d", index, PMT_CACHE_NUM);
		return MT_FTL_FAIL;
	}
	memcpy(param->i4CurrentPMTClusterInCache, &buffer[index],
	       PMT_CACHE_NUM * sizeof(unsigned int));

	mt_ftl_show_param(dev);

	/* Replay */
	ret = mt_ftl_replay(dev);
	if (ret) {
		mt_ftl_err("mt_ftl_replay fail, ret = %d", ret);
		return ret;
	}

	mt_ftl_show_param(dev);

	return ret;
}

int mt_ftl_create(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS;
	int leb = 0, offset = 0, img_reload = 0;
	struct ubi_volume_desc *desc = dev->desc;
	struct mt_ftl_param *param = dev->param;

#ifdef MT_FTL_PROFILE
	memset(profile_time, 0, sizeof(profile_time));
#endif

	/* Allocate buffers for FTL usage */
	ret = mt_ftl_alloc_buffers(dev);
	if (ret)
		return ret;

	/* Check the mapping of CONFIG and REPLAY block */
	ret = ubi_is_mapped(desc, CONFIG_START_BLOCK);
	if (ret == 0) {
		ret = ubi_is_mapped(desc, CONFIG_START_BLOCK + 1);
		if (ret)
			leb = CONFIG_START_BLOCK + 1;
		else {
			mt_ftl_err("leb%d/leb%d are both unmapped", CONFIG_START_BLOCK,
				   CONFIG_START_BLOCK + 1);
			ubi_leb_map(desc, CONFIG_START_BLOCK);
			ubi_leb_map(desc, CONFIG_START_BLOCK + 1);
			leb = CONFIG_START_BLOCK;
		}
	} else {
		leb = CONFIG_START_BLOCK;
	}

	ret = mt_ftl_leb_map(dev, REPLAY_BLOCK);

	/* Check if system.img/usrdata.img is just reloaded */
	img_reload = mt_ftl_check_img_reload(dev, CONFIG_START_BLOCK);
	if (img_reload < 0) {
		mt_ftl_err("mt_ftl_check_img_reload fail, ret = %d", img_reload);
		return MT_FTL_FAIL;
	}

	if (img_reload) {
		mt_ftl_err("system or usrdata image is reloaded");
		ret = mt_ftl_recover_blk(dev);
		if (ret) {
			mt_ftl_err("recover block fail");
			return ret;
		}
	}

	/* Get lastest config page */
	offset = mt_ftl_leb_lastpage_offset(dev, leb);

	if (offset == 0) {
		if ((leb == CONFIG_START_BLOCK) && ubi_is_mapped(desc, CONFIG_START_BLOCK + 1))
			leb = CONFIG_START_BLOCK + 1;
		offset = mt_ftl_leb_lastpage_offset(dev, leb);
		if (offset == 0) {
			mt_ftl_err("Config blocks are empty");
			ret = mt_ftl_param_default(dev);
			if (ret)
				mt_ftl_err("mt_ftl_param_default fail, ret = %d", ret);
			return ret;
		}
	}

	offset -= dev->min_io_size;

	/* Grab configs */
	mt_ftl_err("Get config page, leb:%d, page:%d", leb, offset / dev->min_io_size);
	ret = mt_ftl_leb_read(dev, leb, param->general_page_buffer, offset, dev->min_io_size);
	if (ret)
		return MT_FTL_FAIL;

	/* Sync to backup CONFIG block */
	if (leb == CONFIG_START_BLOCK)
		mt_ftl_write_to_blk(dev, CONFIG_START_BLOCK + 1, param->general_page_buffer);
	else
		mt_ftl_write_to_blk(dev, CONFIG_START_BLOCK, param->general_page_buffer);

	/* Init param */
	ret = mt_ftl_param_init(dev, param->general_page_buffer);
	if (ret) {
		mt_ftl_err("mt_ftl_param_init fail, ret = %d", ret);
		return ret;
	}

	return ret;
}

/* TODO: Tracking remove process to make sure mt_ftl_remove can be called during shut down */
int mt_ftl_remove(struct mt_ftl_blk *dev)
{
	int ret = MT_FTL_SUCCESS;
#ifdef MT_FTL_PROFILE
	int i = 0;

	for (i = 0; i < MT_FTL_PROFILE_TOTAL_PROFILE_NUM; i++)
		mt_ftl_err("%s = %lu ms", mtk_ftl_profile_message[i], profile_time[i] / 1000);
#endif				/* PROFILE */
	mt_ftl_err("Enter");
	mt_ftl_commit(dev);
#ifdef MT_FTL_SUPPORT_COMPR
	crypto_free_comp(dev->param->cc);
#endif
	mt_ftl_err("mt_ftl_commit done");

	return ret;
}
