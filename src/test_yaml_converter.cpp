#include "lamapf_and_gazebo/common_interfaces.h"


std::map<std::string, std::string> yaml_map_path = demoMapYaml;

std::string map_pic_path = demoMapYaml.at("map_path");

int main() {
    // load map
    PictureLoader loader_local(map_pic_path, is_grid_occupied_pgm);

    DimensionLength* dim_local = loader_local.getDimensionInfo();

    auto is_occupied_local = [&loader_local](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };

    std::string yaml_path = yaml_map_path.at("yaml_file_path");

    YAMLMapConfigConverter yaml_converter(yaml_path, dim_local[1]);

    std::cout << "haha" << std::endl;


    auto origin_ptf = yaml_converter.GridToPtfPicYaml(Pointi<2>{0, dim_local[1]-1});

    std::cout << "origin_ptf = " << origin_ptf << std::endl;


    auto origin_pt =  yaml_converter.PtfToGridYaml(Pointf<3>{0, 0, 0});

    std::cout << "origin_pt = " << origin_pt << std::endl;

    reso = yaml_converter.resolution_;
    
    zoom_ratio = std::max(1., ceil(std::min(1000./dim_local[0], 1000./dim_local[1]))); 
    
    Canvas canvas("Yaml visualization", dim_local[0], dim_local[1], 1./reso, zoom_ratio);

    std::cout << "canvas rows / cols = " << canvas.getCanvas().rows << " / " << canvas.getCanvas().cols << std::endl;

    std::cout << "dim_[0] = " << dim_local[0] << ", dim_[1] = " << dim_local[1] << std::endl;

    std::cout << "before zoom_ratio = " << zoom_ratio << ", canvas.resolution_ = " << canvas.resolution_ << std::endl;

    canvas.resolution_ = 1./reso;

    std::cout << "after zoom_ratio = " << zoom_ratio << ", canvas.resolution_ = " << canvas.resolution_ << std::endl;

    bool paused = true;

    std::cout << "haha " << std::endl;

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim_local, is_occupied_local);

        canvas.drawCircleInt(origin_pt[0], origin_pt[1], 2., true, 1, COLOR_TABLE[0]);

        char key = canvas.show();
        if(key == 32) {
            paused = !paused;
        }

    }

    return 0;
}











