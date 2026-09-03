---
trigger: always_on
---

# PLATFORMIO & CLANGD IDE DIAGNOSTICS WORKFLOW

When troubleshooting "undeclared identifier", "file not found", or red squiggly diagnostics in VS Code where `pio run` builds successfully:

## 1. Identify the Active Language Server
- Check `.vscode/settings.json` first:
  - If `"clangd.path"` or `"--compile-commands-dir"` is present, the active language server is **Clangd**, NOT Microsoft C/C++ IntelliSense.
  - Clangd relies strictly on `compile_commands.json`, completely ignoring `.vscode/c_cpp_properties.json`.

## 2. Synchronize Compilation Database
- Check timestamp and content of `compile_commands.json`.
- If new files, directories, or libraries were added/refactored, regenerate it immediately:
  ```powershell
  pio run -t compiledb
  ```
- Verify the new flags (e.g. `-Ilib/utils/include`) are present in `compile_commands.json`.

## 3. Clear Clangd In-Memory Cache
- Clangd does not automatically reload `compile_commands.json` while running.
- Prompt the user to restart the language server:
  - VS Code Command Palette (`Ctrl + Shift + P`) -> `clangd: Restart language server`
  - Or terminate the background `clangd.exe` process so VS Code respawns it with fresh data.

## 4. PlatformIO Library Structure Invariant
- For any custom library in `lib/<library_name>` that places headers under `include/`:
  - Ensure `lib/<library_name>/library.json` includes:
    ```json
    "build": {
      "flags": [
        "-I include"
      ]
    }
    ```
  - Ensure `platformio.ini` has `-I lib/<library_name>/include` in `build_flags` so both compilation and `compiledb` export the include path uniformly.
