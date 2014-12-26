#include <stdio.h>
#include <iostream>

#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QElapsedTimer>

#include "Mesh.h"
#include "OrientHelper.h"
#include "PowerCrustSurfaceReconstruction.h"

int main(int argc, char *argv[])
{
    typedef std::vector<double> Vector3;
    std::vector<Vector3> pnts;
	if(argc < 2) return 0;

	//QElapsedTimer timer; timer.start();
    {
		// Construct surface from point cloud
        PowerCrustSurfaceReconstructionImpl pc;
        pc.pcInit();
        pc.input.readXYZ(argv[1]);
        pc.adapted_main();

		// Add output vertices
        surface_mesh::Mesh m;
        for(auto p : pc.output.GetPoints()) m.add_vertex(Point(p[0], p[1], p[2]));

		// Orient and add faces
		OrientHelper ori;
		for (auto f : ori.reorient(pc.output.GetFaces(), m.n_vertices()))
        {
            std::vector<surface_mesh::Mesh::Vertex> verts;
            for(auto vi : f) verts.push_back(surface_mesh::Mesh::Vertex(vi));
            m.add_face(verts);
        }

		// Turn to pure triangles
        m.triangulate();

		// Output mesh vertices
		for (auto & v : m.vertex_property<Eigen::Vector3d>("v:point").vector())
		{
			std::cout << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
		}

		// Output mesh faces
		for (auto f : m.faces())
		{
			std::cout << "f ";
			for (auto vi : m.vertices(f)) std::cout << (vi.idx() + 1) << " ";
			std::cout << "\n";
		}

        pc.freeAll();
	}

	//printf("%d ms", timer.elapsed());

    return 0;
}

