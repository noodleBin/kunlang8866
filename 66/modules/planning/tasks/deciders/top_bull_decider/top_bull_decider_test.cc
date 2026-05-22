/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/tasks/deciders/top_bull_decider/top_bull_decider.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

namespace century {
namespace planning {

class TopBullDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::TOP_BULL_DECIDER);
    config_.mutable_top_bull_decider_config();
    injector_ = std::make_shared<DependencyInjector>();
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(TopBullDeciderTest, Init) {
  TopBullDecider top_bull_decider(config_, injector_);
  EXPECT_EQ(top_bull_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

TEST_F(TopBullDeciderTest, OppositeDirection_01) {
  TopBullDecider top_bull_decider(config_, injector_);

  AINFO << "TopBull_test: Start test opposite direction 01.";

  std::vector<std::array<double, 3>> path_a;
  path_a.push_back({-2980.416828653, 2147.034693884, -0.131610489});
  path_a.push_back({-2979.425477146, 2146.903460367, -0.131614859});
  path_a.push_back({-2978.434126245, 2146.772222447, -0.131619360});
  path_a.push_back({-2977.442775945, 2146.640979991, -0.131623997});
  path_a.push_back({-2976.451426262, 2146.509732875, -0.131628753});
  path_a.push_back({-2975.460077208, 2146.378481005, -0.131633592});
  path_a.push_back({-2974.468728791, 2146.247224323, -0.131638469});
  path_a.push_back({-2973.477381012, 2146.115962828, -0.131643326});
  path_a.push_back({-2972.486033862, 2145.984696576, -0.131648095});
  path_a.push_back({-2971.494687325, 2145.853425698, -0.131652701});
  path_a.push_back({-2970.503341373, 2145.722150401, -0.131657063});
  path_a.push_back({-2969.511995968, 2145.590870975, -0.131661092});
  path_a.push_back({-2968.520651059, 2145.459587801, -0.131664697});
  path_a.push_back({-2967.529306583, 2145.328301353, -0.131667779});
  path_a.push_back({-2966.537962466, 2145.197012202, -0.131670239});
  path_a.push_back({-2965.546618617, 2145.065721022, -0.131671975});
  path_a.push_back({-2964.555274935, 2144.934428586, -0.131672881});
  path_a.push_back({-2963.563931303, 2144.803135770, -0.131672854});
  path_a.push_back({-2962.572587592, 2144.671843553, -0.131671791});
  path_a.push_back({-2961.581243658, 2144.540553015, -0.131669590});
  path_a.push_back({-2960.589899346, 2144.409265335, -0.131666152});
  path_a.push_back({-2959.598554487, 2144.277981787, -0.131661382});
  path_a.push_back({-2958.607208900, 2144.146703736, -0.131655190});
  path_a.push_back({-2957.615862392, 2144.015432633, -0.131647492});
  path_a.push_back({-2956.624514763, 2143.884170006, -0.131638213});
  path_a.push_back({-2955.633165799, 2143.752917455, -0.131627285});
  path_a.push_back({-2954.641815281, 2143.621676641, -0.131614648});
  path_a.push_back({-2953.650462983, 2143.490449278, -0.131600255});
  path_a.push_back({-2952.659108672, 2143.359237120, -0.131584069});
  path_a.push_back({-2951.667752112, 2143.228041953, -0.131566067});
  path_a.push_back({-2950.676393066, 2143.096865577, -0.131546235});
  path_a.push_back({-2949.685031295, 2142.965709800, -0.131524578});
  path_a.push_back({-2948.693666561, 2142.834576415, -0.131501111});
  path_a.push_back({-2947.702298631, 2142.703467195, -0.131475867});
  path_a.push_back({-2946.710927276, 2142.572383871, -0.131448893});
  path_a.push_back({-2945.719552277, 2142.441328121, -0.131420251});
  path_a.push_back({-2944.728173419, 2142.310301552, -0.131390019});
  path_a.push_back({-2943.736790505, 2142.179305686, -0.131358291});
  path_a.push_back({-2942.745403346, 2142.048341946, -0.131325177});
  path_a.push_back({-2941.754011771, 2141.917411638, -0.131290800});
  path_a.push_back({-2940.762615626, 2141.786515939, -0.131255299});
  path_a.push_back({-2939.771214776, 2141.655655881, -0.131218829});
  path_a.push_back({-2938.779809107, 2141.524832339, -0.131181553});
  path_a.push_back({-2937.788398527, 2141.394046017, -0.131143652});
  path_a.push_back({-2936.796982969, 2141.263297438, -0.131105314});
  path_a.push_back({-2935.805562390, 2141.132586934, -0.131066739});
  path_a.push_back({-2934.814136775, 2141.001914634, -0.131028135});
  path_a.push_back({-2933.822706136, 2140.871280460, -0.130989719});

  std::vector<planning::PathInfo> IGV_A;
  for (const auto& point : path_a) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_A.push_back(path_info);
  }

  std::vector<std::array<double, 3>> path_b;
  path_b.push_back({-2955.021602459, 2143.628778392, 3.001359992});
  path_b.push_back({-2956.011883512, 2143.768102689, 3.002274474});
  path_b.push_back({-2957.002282986, 2143.906531912, 3.003161650});
  path_b.push_back({-2957.992795966, 2144.044103209, 3.004002861});
  path_b.push_back({-2958.983415659, 2144.180867927, 3.004787558});
  path_b.push_back({-2959.974134262, 2144.316885040, 3.005510518});
  path_b.push_back({-2960.964943533, 2144.452216847, 3.006170012});
  path_b.push_back({-2961.955835163, 2144.586926148, 3.006766584});
  path_b.push_back({-2962.946801018, 2144.721074425, 3.007302244});
  path_b.push_back({-2963.937833293, 2144.854720672, 3.007779920});
  path_b.push_back({-2964.928924606, 2144.987920684, 3.008203087});
  path_b.push_back({-2965.920068053, 2145.120726645, 3.008575520});
  path_b.push_back({-2966.911257235, 2145.253186923, 3.008901119});
  path_b.push_back({-2967.902486266, 2145.385346008, 3.009183790});
  path_b.push_back({-2968.893749767, 2145.517244551, 3.009427362});
  path_b.push_back({-2969.885042855, 2145.648919464, 3.009635537});
  path_b.push_back({-2970.876361120, 2145.780404075, 3.009811845});
  path_b.push_back({-2971.867700605, 2145.911728303, 3.009959626});
  path_b.push_back({-2972.859057775, 2146.042918858, 3.010082012});
  path_b.push_back({-2973.850429493, 2146.173999452, 3.010181921});
  path_b.push_back({-2974.841812991, 2146.304991011, 3.010262053});
  path_b.push_back({-2975.833205839, 2146.435911892, 3.010324896});
  path_b.push_back({-2976.824605923, 2146.566778088, 3.010372727});
  path_b.push_back({-2977.816011411, 2146.697603436, 3.010407622});
  path_b.push_back({-2978.807420731, 2146.828399814, 3.010431460});
  path_b.push_back({-2979.798832548, 2146.959177327, 3.010445941});
  path_b.push_back({-2980.790245735, 2147.089944482, 3.010452589});
  path_b.push_back({-2981.781659356, 2147.220708356, 3.010452767});
  path_b.push_back({-2982.773072643, 2147.351474751, 3.010447686});
  path_b.push_back({-2983.764484979, 2147.482248338, 3.010438422});
  path_b.push_back({-2984.755895879, 2147.613032786, 3.010425918});
  path_b.push_back({-2985.747304971, 2147.743830888, 3.010411003});
  path_b.push_back({-2986.738711990, 2147.874644668, 3.010394398});
  path_b.push_back({-2987.730116754, 2148.005475485, 3.010376729});
  path_b.push_back({-2988.721519161, 2148.136324122, 3.010358533});
  path_b.push_back({-2989.712919172, 2148.267190867, 3.010340268});
  path_b.push_back({-2990.704316804, 2148.398075594, 3.010322322});
  path_b.push_back({-2991.695712121, 2148.528977818, 3.010305021});
  path_b.push_back({-2992.687105226, 2148.659896765, 3.010288631});
  path_b.push_back({-2993.678496252, 2148.790831421, 3.010273369});
  path_b.push_back({-2994.669885359, 2148.921780583, 3.010259402});
  path_b.push_back({-2995.661272725, 2149.052742902, 3.010246851});
  path_b.push_back({-2996.652658542, 2149.183716932, 3.010235795});
  path_b.push_back({-2997.644043008, 2149.314701175, 3.010226261});
  path_b.push_back({-2998.635426321, 2149.445694129, 3.010218225});
  path_b.push_back({-2999.626808673, 2149.576694349, 3.010211595});
  path_b.push_back({-3000.618190238, 2149.707700523, 3.010206189});
  path_b.push_back({-3001.609571157, 2149.838711575, 3.010201705});

  std::vector<planning::PathInfo> IGV_B;
  for (const auto& point : path_b) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_B.push_back(path_info);
  }

  // check path relation
  auto path_relation_type = top_bull_decider.AnalyzePathRelation(IGV_A, IGV_B);
  if (PathRelationType::CROSSING_RELATION == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are crossing.";
  } else if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP ==
             path_relation_type) {
    AINFO << "TopBull_test: Path A and B are opposite direction overlap.";
  } else if (PathRelationType::SAME_DIRECTION_OVERLAP == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are same direction overlap.";
  } else {
    AINFO << "TopBull_test: Path A and B are not overlap.";
  }

  AINFO << "TopBull_test: End test opposite direction 01.";
}

TEST_F(TopBullDeciderTest, None_01) {
  TopBullDecider top_bull_decider(config_, injector_);

  AINFO << "TopBull_test: Start test none 01.";

  std::vector<std::array<double, 3>> path_a;
  path_a.push_back({-2503.913877203, 2180.341716219, -0.120450434});
  path_a.push_back({-2502.920920586, 2180.223478370, -0.117790190});
  path_a.push_back({-2501.927623256, 2180.108227804, -0.114642545});
  path_a.push_back({-2500.933935138, 2179.996461288, -0.110982357});
  path_a.push_back({-2499.939814423, 2179.888695264, -0.106797082});
  path_a.push_back({-2498.945220965, 2179.785455105, -0.102081814});
  path_a.push_back({-2497.950122467, 2179.687269711, -0.096836338});
  path_a.push_back({-2496.954495848, 2179.594667840, -0.091063276});
  path_a.push_back({-2495.958324948, 2179.508158589, -0.084806150});
  path_a.push_back({-2494.961633509, 2179.427824952, -0.078700049});
  path_a.push_back({-2493.964487076, 2179.353222511, -0.073088356});
  path_a.push_back({-2492.966954429, 2179.283854683, -0.067980710});
  path_a.push_back({-2491.969095032, 2179.219216498, -0.063384005});
  path_a.push_back({-2490.970963747, 2179.158795707, -0.059304591});
  path_a.push_back({-2489.972607430, 2179.102073410, -0.055748168});
  path_a.push_back({-2488.974068526, 2179.048525208, -0.052719696});
  path_a.push_back({-2487.975385465, 2178.997622165, -0.050223336});
  path_a.push_back({-2486.976591728, 2178.948831737, -0.048262430});
  path_a.push_back({-2485.977716262, 2178.901618676, -0.046839599});
  path_a.push_back({-2484.978788008, 2178.855452105, -0.045943882});
  path_a.push_back({-2483.979829964, 2178.809874149, -0.045467596});
  path_a.push_back({-2482.980856767, 2178.764589673, -0.045234488});
  path_a.push_back({-2481.981875916, 2178.719462095, -0.045099073});
  path_a.push_back({-2480.982890052, 2178.674444823, -0.044997242});
  path_a.push_back({-2479.983900038, 2178.629510584, -0.044922403});
  path_a.push_back({-2478.984907846, 2178.584632584, -0.044874764});
  path_a.push_back({-2477.985913990, 2178.539783761, -0.044854523});
  path_a.push_back({-2476.986919779, 2178.494936930, -0.044861851});
  path_a.push_back({-2475.987926996, 2178.450064743, -0.044896967});
  path_a.push_back({-2474.988936368, 2178.405139558, -0.044960155});
  path_a.push_back({-2473.989949412, 2178.360133453, -0.045051772});
  path_a.push_back({-2472.990967236, 2178.315018094, -0.045172256});
  path_a.push_back({-2471.991991356, 2178.269764695, -0.045322111});
  path_a.push_back({-2470.993023114, 2178.224343928, -0.045501907});
  path_a.push_back({-2469.994063895, 2178.178725877, -0.045712258});
  path_a.push_back({-2468.995115030, 2178.132879987, -0.045953806});
  path_a.push_back({-2467.996178132, 2178.086775085, -0.046227181});
  path_a.push_back({-2466.997254723, 2178.040379663, -0.046531409});
  path_a.push_back({-2465.998342830, 2177.993739133, -0.046673504});
  path_a.push_back({-2464.999411584, 2177.947534413, -0.045642359});
  path_a.push_back({-2464.000363604, 2177.903982043, -0.041555040});
  path_a.push_back({-2463.001043279, 2177.867357573, -0.032335676});
  path_a.push_back({-2462.001346715, 2177.843447904, -0.017009721});
  path_a.push_back({-2461.001388630, 2177.838395468, 0.004498259});
  path_a.push_back({-2460.001631346, 2177.858362646, 0.032187887});
  path_a.push_back({-2459.002998852, 2177.909476316, 0.066051318});
  path_a.push_back({-2458.006989646, 2177.997824625, 0.106164843});
  path_a.push_back({-2457.015813718, 2178.129588650, 0.152831022});

  std::vector<planning::PathInfo> IGV_A;
  for (const auto& point : path_a) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_A.push_back(path_info);
  }

  std::vector<std::array<double, 3>> path_b;
  path_b.push_back({-2453.421184114, 2179.621434935, 0.465805754});
  path_b.push_back({-2452.572556166, 2180.071773477, 0.485052486});
  path_b.push_back({-2451.745001815, 2180.555106605, 0.538826909});
  path_b.push_back({-2450.949630480, 2181.095559864, 0.614423292});
  path_b.push_back({-2450.203575653, 2181.699743258, 0.726004769});
  path_b.push_back({-2449.505787095, 2182.382078742, 0.819557685});
  path_b.push_back({-2448.872656127, 2183.133360820, 0.915131092});
  path_b.push_back({-2448.312047840, 2183.947271631, 1.010923110});
  path_b.push_back({-2447.830026666, 2184.815810382, 1.105837967});
  path_b.push_back({-2447.430551873, 2185.730004828, 1.198552919});
  path_b.push_back({-2447.114843987, 2186.680167016, 1.287186971});
  path_b.push_back({-2446.880963347, 2187.656489352, 1.369904544});
  path_b.push_back({-2446.724557323, 2188.650050808, 1.446066027});
  path_b.push_back({-2446.640460907, 2189.653506599, 1.516190014});
  path_b.push_back({-2446.623493509, 2190.661023524, 1.580854172});
  path_b.push_back({-2446.668543312, 2191.667977832, 1.640447424});
  path_b.push_back({-2446.770537993, 2192.670759263, 1.695237414});
  path_b.push_back({-2446.924559029, 2193.666656277, 1.745611321});
  path_b.push_back({-2447.126114332, 2194.653711525, 1.792134871});
  path_b.push_back({-2447.371254455, 2195.630476478, 1.835371882});
  path_b.push_back({-2447.656486656, 2196.595882709, 1.875680809});
  path_b.push_back({-2447.978540668, 2197.549150737, 1.913194934});
  path_b.push_back({-2448.334299562, 2198.489839581, 1.947993986});
  path_b.push_back({-2448.720759117, 2199.417768014, 1.980103762});
  path_b.push_back({-2449.135011265, 2200.333041109, 2.009487151});
  path_b.push_back({-2449.573764046, 2201.236021991, 2.034953129});
  path_b.push_back({-2450.031385266, 2202.127629569, 2.049606254});
  path_b.push_back({-2450.501113592, 2203.012382116, 2.061392359});

  std::vector<planning::PathInfo> IGV_B;
  for (const auto& point : path_b) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_B.push_back(path_info);
  }

  // check path relation
  auto path_relation_type = top_bull_decider.AnalyzePathRelation(IGV_A, IGV_B);
  if (PathRelationType::CROSSING_RELATION == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are crossing.";
  } else if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP ==
             path_relation_type) {
    AINFO << "TopBull_test: Path A and B are opposite direction overlap.";
  } else if (PathRelationType::SAME_DIRECTION_OVERLAP == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are same direction overlap.";
  } else {
    AINFO << "TopBull_test: Path A and B are not overlap.";
  }

  AINFO << "TopBull_test: End test none 01.";
}

TEST_F(TopBullDeciderTest, SameDirection_01) {
  TopBullDecider top_bull_decider(config_, injector_);
  AINFO << "TopBull_test: start test same direction 01.";

  std::vector<std::array<double, 3>> path_a;
  path_a.push_back({-2927.196221530, 2237.971376160, -0.150484925});
  path_a.push_back({-2926.207518099, 2237.821490562, -0.150424599});
  path_a.push_back({-2925.218806225, 2237.671662093, -0.150368952});
  path_a.push_back({-2924.230086186, 2237.521886128, -0.150317880});
  path_a.push_back({-2923.241358881, 2237.372158257, -0.150271242});
  path_a.push_back({-2922.252624956, 2237.222474205, -0.150228855});
  path_a.push_back({-2921.263885026, 2237.072829893, -0.150190500});
  path_a.push_back({-2920.275139673, 2236.923221484, -0.150155921});
  path_a.push_back({-2919.286389433, 2236.773645414, -0.150124830});
  path_a.push_back({-2918.297634801, 2236.624098418, -0.150096911});
  path_a.push_back({-2917.308876220, 2236.474577559, -0.150071823});
  path_a.push_back({-2916.320114081, 2236.325080248, -0.150049210});
  path_a.push_back({-2915.331348722, 2236.175604258, -0.150028698});
  path_a.push_back({-2914.342580420, 2236.026147737, -0.150009903});
  path_a.push_back({-2913.353809399, 2235.876709219, -0.149992433});
  path_a.push_back({-2912.365035821, 2235.727287627, -0.149975895});
  path_a.push_back({-2911.376259789, 2235.577882272, -0.149959892});
  path_a.push_back({-2910.387481350, 2235.428492857, -0.149944033});
  path_a.push_back({-2909.398700490, 2235.279119466, -0.149927934});
  path_a.push_back({-2908.409917140, 2235.129762557, -0.149911219});
  path_a.push_back({-2907.421131145, 2234.980422949, -0.149893524});
  path_a.push_back({-2906.432342390, 2234.831101821, -0.149874502});
  path_a.push_back({-2905.443550619, 2234.681800674, -0.149853824});
  path_a.push_back({-2904.454755556, 2234.532521323, -0.149831180});
  path_a.push_back({-2903.465956885, 2234.383265877, -0.149806284});
  path_a.push_back({-2902.477154220, 2234.234036705, -0.149778875});
  path_a.push_back({-2901.488347223, 2234.084836430, -0.149748721});
  path_a.push_back({-2900.499535440, 2233.935667874, -0.149715617});
  path_a.push_back({-2899.510718395, 2233.786534041, -0.149679392});
  path_a.push_back({-2898.521895661, 2233.637438097, -0.149639905});
  path_a.push_back({-2897.533066721, 2233.488383313, -0.149597052});
  path_a.push_back({-2896.544231053, 2233.339373040, -0.149550764});
  path_a.push_back({-2895.555388184, 2233.190410687, -0.149501007});
  path_a.push_back({-2894.566537569, 2233.041499659, -0.149447786});
  path_a.push_back({-2893.577678718, 2232.892643347, -0.149391138});
  path_a.push_back({-2892.588811145, 2232.743845080, -0.149331139});
  path_a.push_back({-2891.599934344, 2232.595108080, -0.149267908});
  path_a.push_back({-2890.611047866, 2232.446435438, -0.149201597});
  path_a.push_back({-2889.622151286, 2232.297830073, -0.149132393});
  path_a.push_back({-2888.633244183, 2232.149294694, -0.149060518});
  path_a.push_back({-2887.644326200, 2232.000831776, -0.148986229});
  path_a.push_back({-2886.655397012, 2231.852443524, -0.148909810});
  path_a.push_back({-2885.666456340, 2231.704131842, -0.148831578});
  path_a.push_back({-2884.677503955, 2231.555898307, -0.148751875});
  path_a.push_back({-2883.688539673, 2231.407744143, -0.148671071});
  path_a.push_back({-2882.699563379, 2231.259670195, -0.148589562});
  path_a.push_back({-2881.710575012, 2231.111676909, -0.148507761});
  path_a.push_back({-2880.721574575, 2230.963764312, -0.148426102});

  std::vector<planning::PathInfo> IGV_A;
  for (const auto& point : path_a) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_A.push_back(path_info);
  }

  std::vector<std::array<double, 3>> path_b;
  path_b.push_back({-2902.341020785, 2234.239379162, -0.144914445});
  path_b.push_back({-2901.351527415, 2234.094724904, -0.145638815});
  path_b.push_back({-2900.362219596, 2233.948862657, -0.147217562});
  path_b.push_back({-2899.373165003, 2233.801303348, -0.148991070});
  path_b.push_back({-2898.384368922, 2233.652052695, -0.150599369});
  path_b.push_back({-2897.395785495, 2233.501369688, -0.151871435});
  path_b.push_back({-2896.407365473, 2233.349622214, -0.152752115});
  path_b.push_back({-2895.419044837, 2233.197194214, -0.153253962});
  path_b.push_back({-2894.430775706, 2233.044438949, -0.153425725});
  path_b.push_back({-2893.442508962, 2232.891650818, -0.153332045});
  path_b.push_back({-2892.454214068, 2232.739059193, -0.153040775});
  path_b.push_back({-2891.465864358, 2232.586827126, -0.152615443});
  path_b.push_back({-2890.477443436, 2232.435058978, -0.152111196});
  path_b.push_back({-2889.488944997, 2232.283809898, -0.151573064});
  path_b.push_back({-2888.500365776, 2232.133095215, -0.151035716});
  path_b.push_back({-2887.511708003, 2231.982901006, -0.150524142});
  path_b.push_back({-2886.522976926, 2231.833192932, -0.150054842});
  path_b.push_back({-2885.534179892, 2231.683923980, -0.149637256});
  path_b.push_back({-2884.545324868, 2231.535040629, -0.149275237});
  path_b.push_back({-2883.556420209, 2231.386487749, -0.148968453});
  path_b.push_back({-2882.567473970, 2231.238212087, -0.148713632});
  path_b.push_back({-2881.578493547, 2231.090164630, -0.148505621});
  path_b.push_back({-2880.589485626, 2230.942302051, -0.148338252});
  path_b.push_back({-2879.600455455, 2230.794587267, -0.148205013});
  path_b.push_back({-2878.611407799, 2230.646989674, -0.148099538});
  path_b.push_back({-2877.622346343, 2230.499484717, -0.148015954});
  path_b.push_back({-2876.633273944, 2230.352053323, -0.147949084});
  path_b.push_back({-2875.644192811, 2230.204681154, -0.147894559});
  path_b.push_back({-2874.655104355, 2230.057357757, -0.147848840});
  path_b.push_back({-2873.666009752, 2229.910075815, -0.147809192});
  path_b.push_back({-2872.676909760, 2229.762830345, -0.147773610});
  path_b.push_back({-2871.687804822, 2229.615618015, -0.147740730});
  path_b.push_back({-2870.698695301, 2229.468436583, -0.147709725});
  path_b.push_back({-2869.709581435, 2229.321284410, -0.147680197});
  path_b.push_back({-2868.720463432, 2229.174160082, -0.147652074});
  path_b.push_back({-2867.731341501, 2229.027062138, -0.147625524});
  path_b.push_back({-2866.742215899, 2228.879988876, -0.147600867});
  path_b.push_back({-2865.753086944, 2228.732938235, -0.147578511});
  path_b.push_back({-2864.763954986, 2228.585907730, -0.147558898});
  path_b.push_back({-2863.774820468, 2228.438894449, -0.147542460});
  path_b.push_back({-2862.785683882, 2228.291895077, -0.147529586});
  path_b.push_back({-2861.796545775, 2228.144905948, -0.147520601});
  path_b.push_back({-2860.807406732, 2227.997923116, -0.147515751});
  path_b.push_back({-2859.818267370, 2227.850942436, -0.147515195});
  path_b.push_back({-2858.829128319, 2227.703959657, -0.147519000});
  path_b.push_back({-2857.839990214, 2227.556970511, -0.147527137});
  path_b.push_back({-2856.850853679, 2227.409970804, -0.147539485});
  path_b.push_back({-2855.861719311, 2227.262956513, -0.147555830});

  std::vector<planning::PathInfo> IGV_B;
  for (const auto& point : path_b) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_B.push_back(path_info);
  }

  // check path relation
  auto path_relation_type = top_bull_decider.AnalyzePathRelation(IGV_A, IGV_B);
  if (PathRelationType::CROSSING_RELATION == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are crossing.";
  } else if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP ==
             path_relation_type) {
    AINFO << "TopBull_test: Path A and B are opposite direction overlap.";
  } else if (PathRelationType::SAME_DIRECTION_OVERLAP == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are same direction overlap.";
  } else {
    AINFO << "TopBull_test: Path A and B are not overlap.";
  }

  AINFO << "TopBull_test: End test same direction 01.";
}

TEST_F(TopBullDeciderTest, Cross_01) {
  TopBullDecider top_bull_decider(config_, injector_);
  
  AINFO << "TopBull_test: start test cross 01.";

  std::vector<std::array<double, 3>> path_a;
  path_a.push_back({-2880.110730671, 2155.196555280, -1.721768008});
  path_a.push_back({-2880.261435869, 2154.207996458, -1.722034065});
  path_a.push_back({-2880.412502720, 2153.219492540, -1.722487533});
  path_a.push_back({-2880.564020266, 2152.231054672, -1.722975879});
  path_a.push_back({-2880.715935298, 2151.242670798, -1.723363716});
  path_a.push_back({-2880.868069704, 2150.254316111, -1.723533131});
  path_a.push_back({-2881.020143347, 2149.265944355, -1.723387860});
  path_a.push_back({-2881.171798424, 2148.277501588, -1.722858938});
  path_a.push_back({-2881.322631534, 2147.288925887, -1.721910362});
  path_a.push_back({-2881.472229070, 2146.300156639, -1.720543689});
  path_a.push_back({-2881.620206370, 2145.311140347, -1.718800758});
  path_a.push_back({-2881.766247904, 2144.321836279, -1.716764061});
  path_a.push_back({-2881.910146651, 2143.332217497, -1.714554584});
  path_a.push_back({-2882.051837849, 2142.342282895, -1.712327203});
  path_a.push_back({-2882.191428800, 2141.352054137, -1.710263979});
  path_a.push_back({-2882.329219936, 2140.361577762, -1.708565864});
  path_a.push_back({-2882.465716615, 2139.370928622, -1.707443377});
  path_a.push_back({-2882.601632395, 2138.380205422, -1.707106873});
  path_a.push_back({-2882.737881651, 2137.389533710, -1.707756879});
  path_a.push_back({-2882.875563467, 2136.399065429, -1.709574900});
  path_a.push_back({-2883.015936756, 2135.408979870, -1.712714926});
  path_a.push_back({-2883.160387516, 2134.419484920, -1.717295808});
  path_a.push_back({-2883.310388846, 2133.430819293, -1.723394634});
  path_a.push_back({-2883.467454937, 2132.443253740, -1.731041351});
  path_a.push_back({-2883.633090412, 2131.457090338, -1.740215034});
  path_a.push_back({-2883.808736743, 2130.472661181, -1.750842443});
  path_a.push_back({-2883.995719311, 2129.490322122, -1.762799593});
  path_a.push_back({-2884.195198777, 2128.510443374, -1.775917060});
  path_a.push_back({-2884.408131110, 2127.533398503, -1.789989353});
  path_a.push_back({-2884.635241375, 2126.559549811, -1.804788079});
  path_a.push_back({-2884.877014021, 2125.589235312, -1.820077849});
  path_a.push_back({-2885.133701458, 2124.622757212, -1.835633134});
  path_a.push_back({-2885.405349545, 2123.660374831, -1.851253966});
  path_a.push_back({-2885.691836472, 2122.702302952, -1.866778470});
  path_a.push_back({-2885.992920041, 2121.748715604, -1.882090891});
  path_a.push_back({-2886.308287812, 2120.799754863, -1.897124622});
  path_a.push_back({-2886.637605456, 2119.855543056, -1.911860678});
  path_a.push_back({-2886.980559829, 2118.916197273, -1.926322756});
  path_a.push_back({-2887.336895258, 2117.981844646, -1.940570464});
  path_a.push_back({-2887.706447208, 2117.052639484, -1.954709223});
  path_a.push_back({-2888.089452215, 2116.128898986, -1.969554045});
  path_a.push_back({-2888.490597204, 2115.212909481, -1.993258624});
  path_a.push_back({-2888.923292091, 2114.311445878, -2.032932317});
  path_a.push_back({-2889.401618981, 2113.433416818, -2.088255110});
  path_a.push_back({-2889.930099770, 2112.584587202, -2.143069931});
  path_a.push_back({-2890.498063806, 2111.761609305, -2.187250527});
  path_a.push_back({-2891.098888013, 2110.962282278, -2.225870123});
  path_a.push_back({-2891.727229811, 2110.184382290, -2.259166140});

  std::vector<planning::PathInfo> IGV_A;
  for (const auto& point : path_a) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_A.push_back(path_info);
  }

  std::vector<std::array<double, 3>> path_b;
  path_b.push_back({-2901.641681537, 2136.493933588, -0.099198881});
  path_b.push_back({-2900.646241738, 2136.393759459, -0.101553207});
  path_b.push_back({-2899.651176088, 2136.290807726, -0.104735961});
  path_b.push_back({-2898.656549429, 2136.184437738, -0.108385054});
  path_b.push_back({-2897.662412964, 2136.074346200, -0.112194083});
  path_b.push_back({-2896.668773610, 2135.960500008, -0.115923050});
  path_b.push_back({-2895.675602510, 2135.843069630, -0.119398219});
  path_b.push_back({-2894.682862823, 2135.722368549, -0.122506075});
  path_b.push_back({-2893.690500692, 2135.598795731, -0.125184260});
  path_b.push_back({-2892.698459357, 2135.472791019, -0.127411387});
  path_b.push_back({-2891.706680161, 2135.344799101, -0.129196980});
  path_b.push_back({-2890.715105947, 2135.215243543, -0.130572275});
  path_b.push_back({-2889.723688397, 2135.084509993, -0.131582296});
  path_b.push_back({-2888.732381942, 2134.952934882, -0.132279356});
  path_b.push_back({-2887.741148097, 2134.820801311, -0.132717998});
  path_b.push_back({-2886.749958877, 2134.688339253, -0.132951254});
  path_b.push_back({-2885.758789436, 2134.555727484, -0.133028081});
  path_b.push_back({-2884.767622188, 2134.423099115, -0.132991774});
  path_b.push_back({-2883.776444311, 2134.290547434, -0.132879164});
  path_b.push_back({-2882.785248972, 2134.158132729, -0.132720429});
  path_b.push_back({-2881.794031115, 2134.025888350, -0.132539334});
  path_b.push_back({-2880.802789043, 2133.893827134, -0.132353774});
  path_b.push_back({-2879.811523059, 2133.761946796, -0.132176488});
  path_b.push_back({-2878.820234819, 2133.630234640, -0.132015864});
  path_b.push_back({-2877.828926944, 2133.498671493, -0.131876751});
  path_b.push_back({-2876.837602341, 2133.367234779, -0.131761238});
  path_b.push_back({-2875.846264128, 2133.235900937, -0.131669355});
  path_b.push_back({-2874.854915326, 2133.104647135, -0.131599694});
  path_b.push_back({-2873.863558703, 2132.973452438, -0.131549917});
  path_b.push_back({-2872.872196688, 2132.842298509, -0.131517176});
  path_b.push_back({-2871.880831315, 2132.711169953, -0.131498431});
  path_b.push_back({-2870.889464226, 2132.580054379, -0.131490694});
  path_b.push_back({-2869.898096725, 2132.448942274, -0.131491184});
  path_b.push_back({-2868.906729625, 2132.317826710, -0.131497434});
  path_b.push_back({-2867.915363597, 2132.186703049, -0.131507346});
  path_b.push_back({-2866.923999006, 2132.055568533, -0.131519205});
  path_b.push_back({-2865.932636017, 2131.924421919, -0.131531670});
  path_b.push_back({-2864.941274642, 2131.793263119, -0.131543746});
  path_b.push_back({-2863.949914781, 2131.662092880, -0.131554738});
  path_b.push_back({-2862.958556262, 2131.530912504, -0.131564211});
  path_b.push_back({-2861.967198870, 2131.399723618, -0.131571937});
  path_b.push_back({-2860.975842370, 2131.268527992, -0.131577854});
  path_b.push_back({-2859.984486529, 2131.137327393, -0.131582021});
  path_b.push_back({-2858.993131129, 2131.006123488, -0.131584580});
  path_b.push_back({-2858.001775966, 2130.874917768, -0.131585734});
  path_b.push_back({-2857.010420875, 2130.743711505, -0.131585724});
  path_b.push_back({-2856.019065720, 2130.612505722, -0.131584815});
  path_b.push_back({-2855.027710402, 2130.481301169, -0.131583287});

  std::vector<planning::PathInfo> IGV_B;
  for (const auto& point : path_b) {
    PathInfo path_info;
    path_info.set_x(point[0]);
    path_info.set_y(point[1]);
    path_info.set_theta(point[2]);
    IGV_B.push_back(path_info);
  }

  // check path relation
  auto path_relation_type = top_bull_decider.AnalyzePathRelation(IGV_A, IGV_B);
  if (PathRelationType::CROSSING_RELATION == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are crossing.";
  } else if (PathRelationType::OPPOSITE_DIRECTION_OVERLAP ==
             path_relation_type) {
    AINFO << "TopBull_test: Path A and B are opposite direction overlap.";
  } else if (PathRelationType::SAME_DIRECTION_OVERLAP == path_relation_type) {
    AINFO << "TopBull_test: Path A and B are same direction overlap.";
  } else {
    AINFO << "TopBull_test: Path A and B are not overlap.";
  }

  AINFO << "TopBull_test: End test cross 01.";
}

}  // namespace planning
}  // namespace century
