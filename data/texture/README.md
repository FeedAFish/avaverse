# Convert `fbx` to `ply`

- Load `fbx` into `Blender`.
- Export to `obj` with these options:
  - Path Mode: Copy
  - UV Coordinates.
  - Normals.
  - Materials.
  - Triangulated Mesh
- Extract all texture files inside menu `UV Editing` of `Blender` (Image -> Save as).
- Run the following code

```python
import pymeshlab

ms = pymeshlab.MeshSet()
ms.load_new_mesh("model.obj")
ms.apply_filter("compute_texcoord_transfer_wedge_to_vertex")
ms.save_current_mesh(
    "model.ply",
    binary=False,
    save_vertex_normal=True,
    save_textures=False,
    save_wedge_texcoord=False,
)

```
