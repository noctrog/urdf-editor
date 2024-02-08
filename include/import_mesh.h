#ifndef IMPORT_MESH_H
#define IMPORT_MESH_H

#include <vector>

#include <assimp/scene.h>
#include <raylib.h>

Mesh GenMeshCenteredCylinder(float radius, float height, int slices);

// Function to create a raylib Mesh from an Assimp mesh
Mesh LoadMeshFromAssimp(const aiMesh* mesh);

// Function to load all meshes from a Collada file
std::vector<Mesh> LoadColladaMeshes(const std::string& filename);

Model LoadModelFromCollada(const std::string& filename);


#endif // IMPORT_MESH_H
