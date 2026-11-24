// Export the triangles produced by Gazebo's own mesh loader for planar replay.
#include <iomanip>
#include <iostream>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

int main(int argc, char **argv) {
  if (argc != 2) return 1;
  const auto *mesh = gz::common::MeshManager::Instance()->Load(argv[1]);
  if (!mesh || !mesh->SubMeshCount()) return 1;
  std::cout << std::setprecision(17);
  for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i) {
    const auto sub = mesh->SubMeshByIndex(i).lock();
    if (!sub || sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES ||
        sub->IndexCount() % 3 != 0) return 1;
    for (unsigned int j = 0; j < sub->IndexCount(); ++j) {
      const auto v = sub->Vertex(sub->Index(j));
      std::cout << v.X() << ' ' << v.Y() << ' ' << v.Z() << '\n';
    }
  }
}
