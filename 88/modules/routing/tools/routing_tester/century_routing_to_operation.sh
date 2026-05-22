
source /century/scripts/century_base.sh

cd /century/modules/routing/tools/routing_tester

rm routing_request.pb.txt

cp routing_request_to_operation.pb.txt routing_request.pb.txt

cd /century

./bazel-bin/modules/routing/tools/routing_tester/routing_request

