# Notices

OcarinaVR: Copyright (c) 2026 the ReTraced project. GNU General Public License, version 3 or later ([LICENSE](LICENSE)).

## Downloaded by the installer, not part of this repository

| Part | Where from | Licence |
| --- | --- | --- |
| Hookshot (source, at a fixed revision) | Hookshot's own repository, [git.notrexed.dev/Hookshot/Hookshot](https://git.notrexed.dev/Hookshot/Hookshot) | GPL-3.0-or-later; its third-party parts keep their own notices in its folders |
| Hookshot's build tools ("builder parts": Python and its modules, Vulkan headers and FFmpeg, prebuilt third-party files, wgpu-native, the work files) | Hookshot's releases | Each part's own licence, as Hookshot ships it |
| llvm-mingw (the compiler) | [github.com/mstorsjo/llvm-mingw](https://github.com/mstorsjo/llvm-mingw) | Apache-2.0 with LLVM exception; the mingw-w64 libraries under their own licences |
| OpenXR loader (`openxr_loader.dll`) | Khronos, [github.com/KhronosGroup/OpenXR-SDK](https://github.com/KhronosGroup/OpenXR-SDK) | Apache-2.0 |

Every download is checked against its SHA-256 before it is used.

## In this repository

| Part | Licence |
| --- | --- |
| `src/`, `adapter/`, `tools/`, `hooks.json`, the installer and the shortcuts | GPL-3.0-or-later (this project) |
| `images/logo-*.png` | GPL-3.0-or-later (this project). The lettering is set in IM Fell English SC (Igino Marini, SIL Open Font License 1.1); no font file is included |
| `images/controls.png` | The layout, the texts and the small drawings of the gestures: GPL-3.0-or-later (this project); fonts: IM Fell English SC and Libertinus Sans (SIL Open Font License 1.1). **The line drawings of the two controllers are Meta's** ("Meta Quest Hardware Art" 1.8, Touch Plus controllers, Copyright Facebook Technologies, LLC and its affiliates), used under Meta's [Art Attribution License 1.0](https://developers.meta.com/vr/licenses/art-1.0/) to refer to the product in this mod's manual; only their colours were changed, to show on a dark ground. They are **not** under the GPL: reuse them only under Meta's licence |
| `images/screen-*.jpg` | Screenshots of the game running with this mod; the game's pictures are Nintendo's |

## Ideas, not code

- [BetterVR](https://github.com/Crementif/BotW-BetterVR), the VR mode of *Breath of the Wild* (MIT): the idea of
  Link's hands on the player's, of drawing weapons from the back. Its public page was read; no code was taken.
