//==================================================
// GstMetaCorrespondence.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#include <ait/video/GstMetaCorrespondence.h>

#include <stdexcept>
#include <cstring>
#include <iostream>

#ifndef GST_DISABLE_GST_DEBUG
#define GST_CAT_DEFAULT ensure_debug_category()
static GstDebugCategory *
ensure_debug_category(void)
{
	static gsize cat_gonce = 0;

	if (g_once_init_enter(&cat_gonce)) {
		gsize cat_done;

		cat_done = (gsize)_gst_debug_category_new("correspondence", 0, "correspondence");

		g_once_init_leave(&cat_gonce, cat_done);
	}

	return (GstDebugCategory *)cat_gonce;
}
#else
#define ensure_debug_category() /* NOOP */
#endif /* GST_DISABLE_GST_DEBUG */


/* Correspondence Meta implementation *******************************************/

GType gst_correspondence_meta_api_get_type()
{
	static volatile GType type;
	static const gchar *tags[] = { nullptr };

	if (g_once_init_enter(&type)) {
		GType _type =
			gst_meta_api_type_register("GstCorrespondenceMetaMetaAPI", tags);
		GST_INFO("registering");
		g_once_init_leave(&type, _type);
	}
	return type;
}


static gboolean gst_correspondence_meta_transform(GstBuffer* dest, GstMeta* meta, GstBuffer* buffer, GQuark type, gpointer data)
{
	GstCorrespondenceMeta* smeta = (GstCorrespondenceMeta*)meta;

	if (!gst_buffer_correspondence_meta_has(dest)) {
		GST_DEBUG("copy correspondence metadata");
		GstCorrespondenceMeta* dmeta = gst_buffer_add_correspondence_meta(dest, smeta->id);
		if (dmeta == nullptr) {
			return FALSE;
		}
	}
	else {
		gst_buffer_correspondence_meta_set_id(dest, smeta->id);
	}

	return TRUE;
}

static gboolean gst_correspondence_meta_init(GstMeta* meta, gpointer params, GstBuffer* buffer)
{
	GstCorrespondenceMeta* emeta = (GstCorrespondenceMeta*)meta;
	emeta->id = -1;

	return TRUE;
}

static void gst_correspondence_meta_free(GstMeta* meta, GstBuffer* buffer)
{
	// nothing to do
}

const GstMetaInfo* gst_correspondence_meta_get_info()
{
	static const GstMetaInfo* meta_info = nullptr;

	if (g_once_init_enter(&meta_info)) {
		const GstMetaInfo *mi =
			gst_meta_register(GST_CORRESPONDENCE_META_API_TYPE,
			"GstCorrespondenceMeta",
			sizeof(GstCorrespondenceMeta),
			gst_correspondence_meta_init,
			gst_correspondence_meta_free,
			gst_correspondence_meta_transform);
		g_once_init_leave(&meta_info, mi);
	}
	return meta_info;
}

bool gst_buffer_correspondence_meta_has(GstBuffer* buffer)
{
	return gst_buffer_get_meta(buffer, gst_correspondence_meta_api_get_type()) != nullptr;
}

gint gst_buffer_correspondence_meta_get_id(GstBuffer* buffer)
{
	GstCorrespondenceMeta* meta = (GstCorrespondenceMeta*)gst_buffer_get_meta(buffer, gst_correspondence_meta_api_get_type());
	if (meta == nullptr) {
		std::cerr << "ERROR: No correspondence metadata associdated with Gstreamer buffer" << std::endl;
		return -1;
	}
	return meta->id;
}

void gst_buffer_correspondence_meta_set_id(GstBuffer* buffer, gint id)
{
	GstCorrespondenceMeta* meta = (GstCorrespondenceMeta*)gst_buffer_get_meta(buffer, gst_correspondence_meta_api_get_type());
	if (meta == nullptr) {
		std::cerr << "ERROR: No correspondence metadata associdated with Gstreamer buffer" << std::endl;
	}
	meta->id = id;
}

/**
* gst_buffer_add_correspondence_meta:
* @buffer: a #GstBuffer
* @id: Id to establish correspondence between appsrc and appsink buffers
*
* Attaches #GstCorrespondenceMeta metadata to @buffer with the given
* parameters.
*
* Returns: (transfer none): the #GstCorrespondenceMeta on @buffer.
*/
GstCorrespondenceMeta* gst_buffer_add_correspondence_meta(GstBuffer* buffer, gint id)
{
	GstCorrespondenceMeta* meta;

	g_return_val_if_fail(GST_IS_BUFFER(buffer), nullptr);

	bool ref = false;
	if (GST_OBJECT_REFCOUNT(buffer) > 1) {
		gst_buffer_unref(buffer);
		ref = true;
	}
	meta = (GstCorrespondenceMeta*)gst_buffer_add_meta(buffer, GST_CORRESPONDENCE_META_INFO, NULL);
	if (meta != nullptr) {
		meta->id = id;
	}
	else {
		std::cerr << "ERROR: Could not add metadata to buffer" << std::endl;
	}

	if (ref) {
		gst_buffer_ref(buffer);
	}

	return meta;
}
