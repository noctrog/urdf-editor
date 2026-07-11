#ifndef IMPORT_MESH_H
#define IMPORT_MESH_H

#include <assimp/scene.h>
#include <raylib.h>

#include <vector>

// Generates a closed cylinder centered at the origin.
//
// Args:
//   radius: Radius of the cylinder's circular cross-section.
//   height: Extent along the Z axis.
//   slices: Number of radial segments.
Mesh GenMeshCenteredCylinder(float radius, float height, int slices);

// Converts one Assimp mesh into raylib's mesh representation.
Mesh LoadMeshFromAssimp(const aiMesh* mesh);

// Imports every mesh from a supported model file.
std::vector<Mesh> LoadMeshesFromFile(const std::string& filename);

// Imports a model file and assigns default materials to its meshes.
Model LoadModelFromFile(const std::string& filename);

#endif  // IMPORT_MESH_H
