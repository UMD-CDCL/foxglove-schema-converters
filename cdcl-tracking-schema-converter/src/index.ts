import { ExtensionContext } from "@foxglove/extension";
import { registerSchemaConverters, registerTopicConverters } from "../../cdcl-converters/src/converterRuntime";
import { TRACKING_SCHEMA_CONVERTER_SPECS, TRACKING_TOPIC_CONVERTER_SPECS } from "./specs";

export function activate(extensionContext: ExtensionContext): void {
  registerSchemaConverters(extensionContext, TRACKING_SCHEMA_CONVERTER_SPECS);
  registerTopicConverters(extensionContext, TRACKING_TOPIC_CONVERTER_SPECS);
}
