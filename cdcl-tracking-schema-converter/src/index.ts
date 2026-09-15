import { ExtensionContext } from "@foxglove/extension";
import { registerSchemaConverters } from "../../cdcl-converters/src/converterRuntime";
import { TRACKING_SCHEMA_CONVERTER_SPECS } from "./specs";

export function activate(extensionContext: ExtensionContext): void {
  registerSchemaConverters(extensionContext, TRACKING_SCHEMA_CONVERTER_SPECS);
}
