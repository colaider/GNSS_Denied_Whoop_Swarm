# Formation control research

Research and simulation work for formation control of the GNSS-denied whoop swarm.
The flight stack lives in `../whoopCTRL`, the hardware in `../CAD`.

## Layout

| Folder | What is in it |
|---|---|
| `dvs_simulation/` | 2D Python reimplementation of the Deformable Virtual Structure (DVS): body/world mapping, Voronoi cells clipped to the structure, Lloyd relaxation, Hungarian assignment, plotting. Run `python main.py`. |
| `guides/dvs_math/` | LaTeX guides: the DVS maths (with the split/merge cell-complex extension), homogeneous coordinates for the cell clipping, and self-organising maps. PDFs are built. `figures.py` regenerates the figures. |
| `guides/vla_world_models/` | Background notes on reasoning vision-language-action models and world models, with probability derivations. `make_data.py` regenerates the plot data. Build with `tectonic -X compile vla_wm_notes.tex`. |
| `paper_explanations/` | Plain-language explanations of the papers in the literature survey, grouped by topic. The original papers are not included (copyright); find them by the title in each file name. |
| `simulations/` | `hidden_tesseract_reshuffle.html`: early exploration of a tesseract-projection formation idea (rejected: it fixes the drone count). |

## Requirements

Python 3 with `numpy`, `scipy` and `matplotlib`. The guides build with [tectonic](https://tectonic-typesetting.github.io/).

## Main reference

Zhou, Hou, Xu, Gao (2025), *Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments*, [arXiv:2509.18636](https://arxiv.org/abs/2509.18636).
