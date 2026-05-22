
source /century/scripts/century_base.sh

cd /century

dir_name=/century/modules/map/data/century

./scripts/generate_routing_topo_graph.sh --map_dir ${dir_name}

./bazel-bin/modules/map/tools/sim_map_generator --map_dir=${dir_name} --output_dir=${dir_name}
