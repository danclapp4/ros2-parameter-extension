import { ExtensionContext } from "@foxglove/studio";
import { initParamsPanel } from "./ExamplePanel";

export function activate(extensionContext: ExtensionContext) {
  extensionContext.registerPanel({ name: "Custom Parameters Extenstion", initPanel: initParamsPanel });
}


