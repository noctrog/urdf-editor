#ifndef IMPORT_MESH_H
#define IMPORT_MESH_H

#include <assimp/scene.h>
#include <raylib.h>

#include <vector>

Mesh GenMeshCenteredCylinder(float radius, float height, int slices);

Mesh LoadMeshFromAssimp(const aiMesh* mesh);

std::vector<Mesh> LoadMeshesFromFile(const std::string& filename);

Model LoadModelFromFile(const std::string& filename);

#endif  // IMPORT_MESH_H
