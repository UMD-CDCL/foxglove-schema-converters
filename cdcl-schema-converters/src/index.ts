import { ExtensionContext } from "@foxglove/extension";

import { registerSchemaConverters } from "./generated/converterRuntime";
import { SCHEMA_CONVERTER_SPECS } from "./generated/schemaConverterSpecs";

export function activate(extensionContext: ExtensionContext): void {
  registerSchemaConverters(extensionContext, SCHEMA_CONVERTER_SPECS);
}
