/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Error codes.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 */
#ifndef __IMG_ERRORS__
#define __IMG_ERRORS__

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#define IMG_DBG_ASSERT(expected) ({WARN_ON(!(expected)); 0; })

/* @brief Success */
#define IMG_SUCCESS                                     (0)
/* @brief Timeout */
#define IMG_ERROR_TIMEOUT                               (1)
/* @brief memory allocation failed */
#define IMG_ERROR_MALLOC_FAILED                         (2)
/* @brief Unspecified fatal error */
#define IMG_ERROR_FATAL                                 (3)
/* @brief Memory allocation failed */
#define IMG_ERROR_OUT_OF_MEMORY                         (4)
/* @brief Device is not found */
#define IMG_ERROR_DEVICE_NOT_FOUND                      (5)
/* @brief Device is not available/in use */
#define IMG_ERROR_DEVICE_UNAVAILABLE                    (6)
/* @brief Generic/unspecified failure */
#define IMG_ERROR_GENERIC_FAILURE                       (7)
/* @brief Operation was interrupted - retry */
#define IMG_ERROR_INTERRUPTED                           (8)
/* @brief Invalid id  */
#define IMG_ERROR_INVALID_ID                            (9)
/* @brief A signature value was found to be incorrect */
#define IMG_ERROR_SIGNATURE_INCORRECT                   (10)
/* @brief The provided parameters were inconsistent/incorrect */
#define IMG_ERROR_INVALID_PARAMETERS                    (11)
/* @brief A list/pool has run dry     */
#define IMG_ERROR_STORAGE_TYPE_EMPTY                    (12)
/* @brief A list is full      */
#define IMG_ERROR_STORAGE_TYPE_FULL                     (13)
/* @brief Something has already occurred which the code thinks has not */
#define IMG_ERROR_ALREADY_COMPLETE                      (14)
/* @brief A state machine is in an unexpected/illegal state */
#define IMG_ERROR_UNEXPECTED_STATE                      (15)
/* @brief A required resource could not be created/locked */
#define IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE             (16)
/*
 * @brief An attempt to access a structure/resource was
 * made before it was initialised
 */
#define IMG_ERROR_NOT_INITIALISED                       (17)
/*
 * @brief An attempt to initialise a structure/resource
 * was made when it has already been initialised
 */
#define IMG_ERROR_ALREADY_INITIALISED                   (18)
/* @brief A provided value exceeded stated bounds */
#define IMG_ERROR_VALUE_OUT_OF_RANGE                    (19)
/* @brief The operation has been cancelled */
#define IMG_ERROR_CANCELLED                             (20)
/* @brief A specified minimum has not been met */
#define IMG_ERROR_MINIMUM_LIMIT_NOT_MET                 (21)
/* @brief The requested feature or mode is not supported */
#define IMG_ERROR_NOT_SUPPORTED                         (22)
/* @brief A device or process was idle */
#define IMG_ERROR_IDLE                                  (23)
/* @brief A device or process was busy */
#define IMG_ERROR_BUSY                                  (24)
/* @brief The device or resource has been disabled */
#define IMG_ERROR_DISABLED                              (25)
/* @brief The requested operation is not permitted at this time */
#define IMG_ERROR_OPERATION_PROHIBITED                  (26)
/* @brief The entry read from the MMU page directory is invalid */
#define IMG_ERROR_MMU_PAGE_DIRECTORY_FAULT              (27)
/* @brief The entry read from an MMU page table is invalid */
#define IMG_ERROR_MMU_PAGE_TABLE_FAULT                  (28)
/* @brief The entry read from an MMU page catalogue is invalid */
#define IMG_ERROR_MMU_PAGE_CATALOGUE_FAULT              (29)
/* @brief Memory can not be freed as it is still been used */
#define IMG_ERROR_MEMORY_IN_USE                         (30)
/* @brief A mismatch has unexpectedly occurred in data */
#define IMG_ERROR_TEST_MISMATCH                         (31)

#define IMG_ERROR_INVALID_CONTEXT			(32)

#define IMG_ERROR_RETRY					(33)
#define IMG_ERROR_UNDEFINED				(34)
#define IMG_ERROR_INVALID_SIZE				(35)
#define IMG_ERROR_SURFACE_LOCKED			(36)

/* Mutex subclasses */
#define SUBCLASS_BASE       0
#define SUBCLASS_VXD_V4L2   1
#define SUBCLASS_VXE_V4L2   1
#define SUBCLASS_BSPP       1
#define SUBCLASS_ADDR_ALLOC 7
#define SUBCLASS_IMGMEM     6
#define SUBCLASS_RMAN       1
#define SUBCLASS_TALMMU     5
#define SUBCLASS_VXD_CORE   2
#define SUBCLASS_POOL       3
#define SUBCLASS_POOL_RES   5
#define SUBCLASS_TOPAZ_API  2
#define SUBCLASS_TOPAZDD_TX 4
#define SUBCLASS_TOPAZDD    3

#endif /* __IMG_ERRORS__ */
