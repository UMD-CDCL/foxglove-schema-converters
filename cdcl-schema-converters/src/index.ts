import { ExtensionContext } from "@foxglove/extension";

import { registerGeneratedSchemaConverters } from "./generatedSchemaConverters";

export function activate(extensionContext: ExtensionContext): void {
  registerGeneratedSchemaConverters(extensionContext);
}
