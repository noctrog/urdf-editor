#include <import_mesh.h>
#include <external/par_shapes.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <loguru.hpp>

::Mesh GenMeshCenteredCylinder(float radius, float height, int slices)
{
    ::Mesh mesh = { 0 };

    if (slices >= 3)
    {
        // Create a cylinder along Y-axis
        par_shapes_mesh* cylinder = par_shapes_create_cylinder(slices, 8);
        par_shapes_scale(cylinder, radius, radius, height);

        // Translate the cylinder to center it at the origin
        par_shapes_translate(cylinder, 0, 0, -height / 2.0F);

        // Create top cap
        float center_top[] = { 0, 0, height / 2 };
        float normal_top[] = { 0, 0, 1 };
        par_shapes_mesh* cap_top = par_shapes_create_disk(radius, slices, center_top, normal_top);
        cap_top->tcoords = PAR_MALLOC(float, 2 * cap_top->npoints);
        for (int i = 0; i < 2 * cap_top->npoints; i++) cap_top->tcoords[i] = 0.0F;

        // Create bottom cap
        float center_bot[] = { 0, 0, 0 };
        float normal_bot[] = { 0, 0, 1 };
        par_shapes_mesh* cap_bottom = par_shapes_create_disk(radius, slices, center_bot, normal_bot);
        float x_vec[] = {1, 0, 0};
        par_shapes_rotate(cap_bottom, -PI, x_vec);
        par_shapes_translate(cap_bottom, 0, 0, -height / 2.0F);
        cap_bottom->tcoords = PAR_MALLOC(float, 2 * cap_bottom->npoints);
        for (int i = 0; i < 2 * cap_bottom->npoints; i++) cap_bottom->tcoords[i] = 0.95F;

        // Merge cylinder and caps
        par_shapes_merge_and_free(cylinder, cap_top);
        par_shapes_merge_and_free(cylinder, cap_bottom);


        mesh.vertices = (float *)RL_MALLOC(cylinder->ntriangles*3*3*sizeof(float));
        mesh.texcoords = (float *)RL_MALLOC(cylinder->ntriangles*3*2*sizeof(float));
        mesh.normals = (float *)RL_MALLOC(cylinder->ntriangles*3*3*sizeof(float));

        mesh.vertexCount = cylinder->ntriangles*3;
        mesh.triangleCount = cylinder->ntriangles;

        for (int k = 0; k < mesh.vertexCount; k++)
        {
            mesh.vertices[k*3] = cylinder->points[cylinder->triangles[k]*3];
            mesh.vertices[k*3 + 1] = cylinder->points[cylinder->triangles[k]*3 + 1];
            mesh.vertices[k*3 + 2] = cylinder->points[cylinder->triangles[k]*3 + 2];

            mesh.normals[k*3] = cylinder->normals[cylinder->triangles[k]*3];
            mesh.normals[k*3 + 1] = cylinder->normals[cylinder->triangles[k]*3 + 1];
            mesh.normals[k*3 + 2] = cylinder->normals[cylinder->triangles[k]*3 + 2];

            mesh.texcoords[k*2] = cylinder->tcoords[cylinder->triangles[k]*2];
            mesh.texcoords[k*2 + 1] = cylinder->tcoords[cylinder->triangles[k]*2 + 1];
        }

        par_shapes_free_mesh(cylinder);

        // Upload vertex data to GPU (static mesh)
        UploadMesh(&mesh, false);
    }
    else LOG_F(WARNING, "MESH: Failed to generate mesh: cylinder");

    return mesh;
}



// Function to create a raylib Mesh from an Assimp mesh
Mesh LoadMeshFromAssimp(const aiMesh* mesh) {
    Mesh result = { 0 };

    // Allocate vertex and index buffers
    result.vertexCount = mesh->mNumVertices;
    result.vertices = (float*)RL_MALLOC(result.vertexCount * 3 * sizeof(float)); // Assuming 3D vertices
    result.triangleCount = mesh->mNumFaces;
    result.indices = (unsigned short*)RL_MALLOC(result.triangleCount * 3 * sizeof(unsigned short));  // Assuming triangles

    // Copy vertex positions
    memcpy(result.vertices, mesh->mVertices, result.vertexCount * 3 * sizeof(float));

    // Copy indices
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        const aiFace* face = &mesh->mFaces[i];
        memcpy(&result.indices[i * 3], face->mIndices, 3 * sizeof(unsigned int));
    }

    return result;
}

// Function to load all meshes from a Collada file
std::vector<Mesh> LoadColladaMeshes(const std::string& filename) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        LOG_F(WARNING, "ERROR: %s", importer.GetErrorString());
        return std::vector<Mesh>(1, Mesh());
    }

    std::vector<Mesh> meshes;
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        meshes.push_back(LoadMeshFromAssimp(scene->mMeshes[i]));
    }

    importer.FreeScene();
    return meshes;
}

Model LoadModelFromCollada(const std::string& filename) {
    Model model = {};
    std::vector<Mesh> meshes = LoadColladaMeshes(filename);
    model.meshCount = meshes.size();
    model.meshes = (Mesh*)RL_MALLOC(model.meshCount * sizeof(Mesh));
    for (size_t i = 0; i < meshes.size(); i++) {
        model.meshes[i] = meshes[i];
    }
    return model;
}
