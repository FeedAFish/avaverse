# Convert `fbx` to `ply`

- Load `fbx` into `Blender`.
- Export to `obj` with these options:
  - Path Mode: Copy
  - UV Coordinates.
  - Normals.
  - Materials.
  - Triangulated Mesh
- Extract all texture files inside menu `UV Editing` of `Blender` (Image -> Save as).
- Re-import the `obj` file into `Blender` and follow this [tutorial](https://github.com/rlguy/Blender-FLIP-Fluids/wiki/Manifold-Meshes#addon-3d-print-toolbox) to make manifold.
- Re-export `obj` with the same option as above.
- Run the following code

```python
import pymeshlab

ms = pymeshlab.MeshSet()
ms.load_new_mesh("model.obj")
ms.apply_filter("compute_texcoord_transfer_wedge_to_vertex")
ms.apply_filter("compute_matrix_from_rotation", angle=90)
ms.save_current_mesh(
    "model.ply",
    binary=False,
    save_vertex_color=False,
    save_vertex_normal=True,
    save_textures=False,
    save_face_color=False,
    save_wedge_texcoord=False,
)
```
