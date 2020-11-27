#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"

// Test Autocomplete function
TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test the lower case
  auto names = m.Autocomplete("c");
  std::vector<std::string> gt1 = {"Crosswalk1", "CVS", "Community of Christ", "Carson Center", "ChickfilA", "Coffee Bean1", "Cava", "Century Apartments 1",
                                  "CorePower Yoga", "Century 2", "Cardinal Gardens", "Cal Mart Beer 38 Wine Food Store", "Coffee Bean2", "Car Pooling station",
                                  "crosswalk3", "Crosswalk2", "Chipotle Mexican Grill"};
  EXPECT_EQ(names, gt1);

  // Test the Upper case
  names = m.Autocomplete("C");
  EXPECT_EQ(names, gt1);

  // Test non-exist names
  names = m.Autocomplete("rr"); 
  std::vector<std::string> gt3 = {}; 
  EXPECT_EQ(names, gt3);

  // Test  mix lower and upper case
  names = m.Autocomplete("GeO");
  std::vector<std::string> gt4 = {"George Lucas Instructional Building", "George Tirebiter"};
  EXPECT_EQ(names, gt4);

  // Test  empty inpute
  names = m.Autocomplete(" ");
  std::vector<std::string> gt5 = {};
  EXPECT_EQ(names, gt5);
}

// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test Cava
  auto position = m.GetPosition("Cava");
  std::pair<double, double> gt1(34.0250185, -118.2844838);
  EXPECT_EQ(position, gt1);
  // Test Bank of America
  position = m.GetPosition("Bank of America");
  std::pair<double, double> gt2(34.025187, -118.2841713);
  EXPECT_EQ(position, gt2);
  // Test Expo ParkUSC
  position = m.GetPosition("Expo ParkUSC");
  std::pair<double, double> gt3(34.0182548, -118.2857016);
  EXPECT_EQ(position, gt3);
  // Test Lyons Center
  position = m.GetPosition("Lyons Center");
  std::pair<double, double> gt4(34.0244304, -118.2884699);
  EXPECT_EQ(position, gt4);
  // Test non-exist name
  position = m.GetPosition("rrrrrrr");
  std::pair<double, double> gt5(-1, -1);
  EXPECT_EQ(position, gt5);
}

// Test CalculateShortestPath dijkstra
TEST(TrojanMapTest, CalculateShortestPath) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to ChickfilA
  auto path = m.CalculateShortestPath("Lyons Center", "Expo ParkUSC");
  std::vector<std::string> gt{"6042952605", "6808200832", "6352860083", "1286136440", "21098539",
                              "6389467809", "4015203132", "3195897587", "4015203128", "4015203123",
                              "1878000351", "6814958436", "4015203121", "4015203120", "4872897506", 
                              "6814958395", "544672064", "6814958399", "4015203116", "4015203113",
                              "4015203110", "123152312", "6814955793", "4015372472", "4015372470", 
                              "4015372465", "4015372464", "2613159968", "6813379562", "63068674",
                              "6813379574", "4015405548", "4015405547", "7023430187", "7217510383",
                              "1732243601", "4015405546", "1732243668", "7023430190", "1732243673",
                              "7023424980", "4399693644"};
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to ChickfilA
  path = m.CalculateShortestPath("Expo ParkUSC", "Lyons Center");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // empty check
  path = m.CalculateShortestPath("rrr", "xxxx");
  gt = {};
   EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath Bellman
TEST(TrojanMapTest, CalculateShortestPathBellman) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to ChickfilA
  auto path = m.CalculateShortestPathBellman("Lyons Center", "Expo ParkUSC");
  std::vector<std::string> gt{"6042952605", "6808200832", "6352860083", "1286136440", "21098539",
                              "6389467809", "4015203132", "3195897587", "4015203128", "4015203123",
                              "1878000351", "6814958436", "4015203121", "4015203120", "4872897506", 
                              "6814958395", "544672064", "6814958399", "4015203116", "4015203113",
                              "4015203110", "123152312", "6814955793", "4015372472", "4015372470", 
                              "4015372465", "4015372464", "2613159968", "6813379562", "63068674",
                              "6813379574", "4015405548", "4015405547", "7023430187", "7217510383",
                              "1732243601", "4015405546", "1732243668", "7023430190", "1732243673",
                              "7023424980", "4399693644"};
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to ChickfilA
  path = m.CalculateShortestPathBellman("Expo ParkUSC", "Lyons Center");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // empty check
  path = m.CalculateShortestPathBellman("rrr", "xxxx");
  gt = {};
   EXPECT_EQ(path, gt);
}

// 2-OPT testing
TEST(TrojanMapTest, TSP) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1873056015", "6905329551", "213332060", "1931345270"}; // Input location ids 
  auto result = m.TravellingTrojan2_Opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1873056015", "213332060", "1931345270", "6905329551", "1873056015"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
