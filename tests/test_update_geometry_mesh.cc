#include <command.h>
#include <doctest/doctest.h>
#include <raymath.h>

#include <filesystem>
#include <memory>

TEST_CASE("UpdateGeometryMeshCommand preserves the model when the new file is missing") {
    MaterialMap maps[1]{};
    ::Material materials[1]{};
    materials[0].maps = maps;

    Model model{};
    model.materialCount = 1;
    model.materials = materials;
    model.transform = MatrixIdentity();

    auto mesh = std::make_shared<urdf::Mesh>("original.stl", "original.stl");
    Shader shader{};
    const auto missing_path =
        std::filesystem::temp_directory_path() / "urdf-editor-missing-mesh.stl";

    UpdateGeometryMeshCommand command(mesh, missing_path.string(), model, shader);
    command.execute();

    CHECK(model.materials == materials);
    CHECK(mesh->filename == "original.stl");
    CHECK(mesh->resolved_path == "original.stl");

    command.undo();
    CHECK(model.materials == materials);
    CHECK(mesh->filename == "original.stl");
}
