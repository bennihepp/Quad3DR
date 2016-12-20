//==================================================
// GstMetaCorrespondence.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#pragma once

#include <gst/gst.h>

G_BEGIN_DECLS

/**
* GstCorrespondenceMeta:
* @meta: parent #GstMeta
* @id: Id to establish correspondence between appsrc and appsink buffers
*/
typedef struct {
	GstMeta meta;

	gint id;
} GstCorrespondenceMeta;

GType gst_correspondence_meta_api_get_type();
#define GST_CORRESPONDENCE_META_API_TYPE (gst_correspondence_meta_api_get_type())
const GstMetaInfo *gst_correspondence_meta_get_info();
#define GST_CORRESPONDENCE_META_INFO (gst_correspondence_meta_get_info())

#define gst_buffer_get_correspondence_meta(b) \
        ((GstCorrespondenceMeta*)gst_buffer_get_meta((b), GST_CORRESPONDENCE_META_API_TYPE))
GstCorrespondenceMeta *gst_buffer_add_correspondence_meta(GstBuffer* buffer, gint id);

bool gst_buffer_correspondence_meta_has(GstBuffer* buffer);
gint gst_buffer_correspondence_meta_get_id(GstBuffer* buffer);
void gst_buffer_correspondence_meta_set_id(GstBuffer* buffer, gint id);

G_END_DECLS
