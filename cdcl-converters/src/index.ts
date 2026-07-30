import { ExtensionContext } from "@foxglove/extension";

import { registerSchemaConverters, registerTopicConverters } from "./converterRuntime";
import { SCHEMA_CONVERTER_SPECS, TOPIC_CONVERTER_SPECS } from "./converterSpecs";

export function activate(extensionContext: ExtensionContext): void {
  registerSchemaConverters(extensionContext, SCHEMA_CONVERTER_SPECS);
  registerTopicConverters(extensionContext, TOPIC_CONVERTER_SPECS);
}
