import { ExtensionContext } from "@foxglove/extension";

import { registerTopicConverters } from "./generated/converterRuntime";
import { TOPIC_CONVERTER_SPECS } from "./generated/topicConverterSpecs";

export function activate(extensionContext: ExtensionContext): void {
  registerTopicConverters(extensionContext, TOPIC_CONVERTER_SPECS);
}
