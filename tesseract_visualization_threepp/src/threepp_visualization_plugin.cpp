/**
 * @file threepp_visualization_plugin.cpp
 * @brief Exports a factory of ThreeppVisualization for tesseract's VisualizationLoader
 *
 * Load it by setting
 *   TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
 *   TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threepp_plugin
 */
#include <memory>
#include <string>

#include <tesseract/common/property_tree.h>
#include <tesseract/visualization/threepp/threepp_visualization.h>
#include <tesseract/visualization/threepp/threepp_visualization_anchor.h>
#include <tesseract/visualization/visualization_factory.h>

namespace tesseract::visualization::threepp
{
/** @brief Optional configuration: mode, width, height and lighting. Unset values come from the environment */
class ThreeppVisualizationFactory : public tesseract::visualization::VisualizationFactory
{
public:
  tesseract::common::PropertyTree schema() const override
  {
    using namespace tesseract::common;
    // clang-format off
    return PropertyTreeBuilder()
        .attribute(property_attribute::TYPE, property_type::CONTAINER)
        .string("mode").done()
        .int32("width").minimum(1).done()
        .int32("height").minimum(1).done()
        .string("lighting").done()
        .build();
    // clang-format on
  }

protected:
  std::unique_ptr<tesseract::visualization::Visualization>
  createImpl(const std::string& /*name*/, const tesseract::common::PropertyTree& config) const override
  {
    ThreeppVisualizationOptions options = ThreeppVisualizationOptions::fromEnvironment();
    if (const auto* mode = config.find("mode"); mode != nullptr && !mode->isNull())
      options.mode = ThreeppVisualizationOptions::parseMode(mode->as<std::string>());
    if (const auto* width = config.find("width"); width != nullptr && !width->isNull())
      options.width = width->as<int>();
    if (const auto* height = config.find("height"); height != nullptr && !height->isNull())
      options.height = height->as<int>();
    if (const auto* lighting = config.find("lighting"); lighting != nullptr && !lighting->isNull())
      options.lighting = ThreeppVisualizationOptions::parseLighting(lighting->as<std::string>());
    return std::make_unique<ThreeppVisualization>(std::move(options));
  }
};

PLUGIN_ANCHOR_IMPL(ThreeppVisualizationAnchor)
}  // namespace tesseract::visualization::threepp

TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::threepp::ThreeppVisualizationFactory,
                                   TesseractThreeppVisualizationPlugin)

// VisualizationLoader::get() with no name asks for the Ignition symbol, as with the three.js plugin.
// If more than one plugin exporting it is listed, library search order decides which loads.
TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::threepp::ThreeppVisualizationFactory,
                                   TesseractIgnitionVisualizationPlugin)
