/**
 * @file threepp_visualization_anchor.h
 * @brief Anchor for packaged builds. Calling it links the plugin library into the caller, which is
 * what makes wheel repair tools (auditwheel, delvewheel, delocate) bundle the plugin.
 */
#ifndef TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_ANCHOR_H
#define TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_ANCHOR_H

#include <boost_plugin_loader/macros.h>

namespace tesseract::visualization::threepp
{
PLUGIN_ANCHOR_DECL(ThreeppVisualizationAnchor)
}

#endif  // TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_ANCHOR_H
