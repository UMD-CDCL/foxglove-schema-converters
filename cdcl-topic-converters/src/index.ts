import { ExtensionContext } from "@foxglove/extension";

import { registerGeneratedTbaTopicConverters } from "./generatedTbaTopicConverters";

export function activate(extensionContext: ExtensionContext): void {
  registerGeneratedTbaTopicConverters(extensionContext);
}
