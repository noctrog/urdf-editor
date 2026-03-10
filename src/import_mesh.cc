#include <assimp/postprocess.h>
#include <external/par_shapes.h>
#include <import_mesh.h>

#include <assimp/Importer.hpp>
#include <loguru.hpp>

::Mesh GenMeshCenteredCylinder(float radius, float height, int slices) {
    ::Mesh mesh = {0};

    if (slices >= 3) {
        // Create a cylinder along Y-axis
        par_shapes_mesh* cylinder = par_shapes_create_cylinder(slices, 8);
        par_shapes_scale(cylinder, radius, radius, height);

        // Translate the cylinder to center it at the origin
        par_shapes_translate(cylinder, 0, 0, -height / 2.0F);

        // Create top cap
        float center_top[] = {0, 0, height / 2};
        float normal_top[] = {0, 0, 1};
        par_shapes_mesh* cap_top = par_shapes_create_disk(radius, slices, center_top, normal_top);
        cap_top->tcoords = PAR_MALLOC(float, 2 * cap_top->npoints);
        for (int i = 0; i < 2 * cap_top->npoints; i++) cap_top->tcoords[i] = 0.0F;

        // Create bottom cap
        float center_bot[] = {0, 0, 0};
        float normal_bot[] = {0, 0, 1};
        par_shapes_mesh* cap_bottom =
            par_shapes_create_disk(radius, slices, center_bot, normal_bot);
        float x_vec[] = {1, 0, 0};
        par_shapes_rotate(cap_bottom, -PI, x_vec);
        par_shapes_translate(cap_bottom, 0, 0, -height / 2.0F);
        cap_bottom->tcoords = PAR_MALLOC(float, 2 * cap_bottom->npoints);
        for (int i = 0; i < 2 * cap_bottom->npoints; i++) cap_bottom->tcoords[i] = 0.95F;

        // Merge cylinder and caps
        par_shapes_merge_and_free(cylinder, cap_top);
        par_shapes_merge_and_free(cylinder, cap_bottom);

        mesh.vertices = (float*)RL_MALLOC(cylinder->ntriangles * 3 * 3 * sizeof(float));
        mesh.texcoords = (float*)RL_MALLOC(cylinder->ntriangles * 3 * 2 * sizeof(float));
        mesh.normals = (float*)RL_MALLOC(cylinder->ntriangles * 3 * 3 * sizeof(float));

        mesh.vertexCount = cylinder->ntriangles * 3;
        mesh.triangleCount = cylinder->ntriangles;

        for (int k = 0; k < mesh.vertexCount; k++) {
            mesh.vertices[k * 3] = cylinder->points[cylinder->triangles[k] * 3];
            mesh.vertices[k * 3 + 1] = cylinder->points[cylinder->triangles[k] * 3 + 1];
            mesh.vertices[k * 3 + 2] = cylinder->points[cylinder->triangles[k] * 3 + 2];

            mesh.normals[k * 3] = cylinder->normals[cylinder->triangles[k] * 3];
            mesh.normals[k * 3 + 1] = cylinder->normals[cylinder->triangles[k] * 3 + 1];
            mesh.normals[k * 3 + 2] = cylinder->normals[cylinder->triangles[k] * 3 + 2];

            mesh.texcoords[k * 2] = cylinder->tcoords[cylinder->triangles[k] * 2];
            mesh.texcoords[k * 2 + 1] = cylinder->tcoords[cylinder->triangles[k] * 2 + 1];
        }

        par_shapes_free_mesh(cylinder);

        // Upload vertex data to GPU (static mesh)
        UploadMesh(&mesh, false);
    } else
        LOG_F(WARNING, "MESH: Failed to generate mesh: cylinder");

    return mesh;
}

Mesh LoadMeshFromAssimp(const aiMesh* mesh) {
    Mesh result = {0};

    result.vertexCount = mesh->mNumVertices;
    result.vertices = (float*)RL_MALLOC(result.vertexCount * 3 * sizeof(float));
    result.triangleCount = mesh->mNumFaces;
    result.indices = (unsigned short*)RL_MALLOC(result.triangleCount * 3 * sizeof(unsigned short));

    memcpy(result.vertices, mesh->mVertices, result.vertexCount * 3 * sizeof(float));

    if (mesh->HasNormals()) {
        result.normals = (float*)RL_MALLOC(result.vertexCount * 3 * sizeof(float));
        memcpy(result.normals, mesh->mNormals, result.vertexCount * 3 * sizeof(float));
    }

    if (mesh->HasTextureCoords(0)) {
        result.texcoords = (float*)RL_MALLOC(result.vertexCount * 2 * sizeof(float));
        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            result.texcoords[i * 2]     = mesh->mTextureCoords[0][i].x;
            result.texcoords[i * 2 + 1] = mesh->mTextureCoords[0][i].y;
        }
    }

    // Convert from assimp's unsigned int to raylib's unsigned short
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        const aiFace* face = &mesh->mFaces[i];
        for (int j = 0; j < 3; j++) {
            result.indices[i * 3 + j] = static_cast<unsigned short>(face->mIndices[j]);
        }
    }

    return result;
}

std::vector<Mesh> LoadMeshesFromFile(const std::string& filename) {
    Assimp::Importer importer;
    const aiScene* scene =
        importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_PreTransformVertices);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        LOG_F(WARNING, "Failed to load mesh file '%s': %s", filename.c_str(),
              importer.GetErrorString());
        return std::vector<Mesh>(1, Mesh());
    }

    LOG_F(1, "Loaded mesh file '%s' with %d meshes", filename.c_str(), scene->mNumMeshes);

    std::vector<Mesh> meshes;
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        meshes.push_back(LoadMeshFromAssimp(scene->mMeshes[i]));
    }

    importer.FreeScene();
    return meshes;
}

Model LoadModelFromFile(const std::string& filename) {
    Model model = {};
    std::vector<Mesh> meshes = LoadMeshesFromFile(filename);
    model.meshCount = meshes.size();
    model.meshes = (Mesh*)RL_MALLOC(model.meshCount * sizeof(Mesh));
    for (size_t i = 0; i < meshes.size(); i++) {
        model.meshes[i] = meshes[i];
        UploadMesh(&model.meshes[i], false);
    }

    model.materialCount = 1;
    model.materials = (Material*)RL_MALLOC(sizeof(Material));
    model.materials[0] = LoadMaterialDefault();

    // Assign default material to all meshes
    model.meshMaterial = (int*)RL_MALLOC(model.meshCount * sizeof(int));
    for (int i = 0; i < model.meshCount; i++) {
        model.meshMaterial[i] = 0;
    }

    return model;
}
