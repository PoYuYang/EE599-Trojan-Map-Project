#include "trojanmap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {
  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto results = CalculateShortestPath(input1, input2);
    // auto results = CalculateShortestPathBellman(input1, input2);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    // locations = {"6226313827", "63765376", "1838284627", "441893821", "6807374562", "6807221803"};
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    // auto results = TravellingTrojan(locations);
    // auto results = TravellingTrojanBruteForceImprovement(locations);
    auto results = TravellingTrojan2_Opt(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
    break;
  default:
    std::cout << "Please select 1 - 5." << std::endl;
    PrintMenu();
    break;
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
  this->SaveLocName();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) { return this->data[id].lat; }

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { return this->data[id].lon; }

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { return this->data[id].name; }

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    std::vector<std::string> result = this->data[id].neighbors;
    return result;
}


/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) {
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  double dlon = b.lon - a.lon;
  double dlat = b.lat - a.lat;
  double Haversine = pow(sin(toRad(dlat / 2)), 2) + cos(toRad(a.lat)) * cos(toRad(b.lat)) * pow(toRad(sin(dlon / 2)), 2);
  Haversine = sqrt(Haversine);
  double c = 2 * asin(std::min(1.0, Haversine));
  double dis = 3961.0 * c;
  return dis;
}

double TrojanMap::toRad(double degree) {
  return degree/180 * M_PI;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.5
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  double total = 0.0;
  for(int i = 1; i < path.size(); ++i) {
    double temp = CalculateDistance(data[path[i - 1]], data[path[i]]);
    total += temp;
  }
  return total;
}

void TrojanMap::SaveLocName() {
  for(auto it:data) {
    if(it.second.name != "") {
      this->name_id[it.second.name] = it.first;
    }
  }
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  // Time complexity for std::transform function: O(input.size())
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);  
  int n = name.size();
  
  for(auto it:data) {
    std::string str = it.second.name;
    // Time complexity for std::transform function: O(str.size())
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    if(str.substr(0, n) == name) results.push_back(it.second.name);
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  if(this->name_id.find(name) != this->name_id.end()) {
    results.first = this->data[this->name_id[name]].lat;
    results.second = this->data[this->name_id[name]].lon;
  }
  return results;
}

/**
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 * Dijkstra Alogrithms implement
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath(std::string location1_name, std::string location2_name) {
  // Invaild input return
  if(this->name_id.find(location1_name) == this->name_id.end()) return {};
  if(this->name_id.find(location2_name) == this->name_id.end()) return {};
  // Turn name into ID
  std::string loc1_id = this->name_id[location1_name];
  std::string loc2_id = this->name_id[location2_name];
  // Output
  std::vector<std::string> path;
  std::unordered_map<std::string, bool> visited;
  // Save the dis table: key: ID, value: distance to loc1
  std::unordered_map<std::string, double> dis;
  // Save the relationship(for backtracking): key: node ID, value: it's parent
  std::unordered_map<std::string, std::string> parent;
  // Intialize dis ansd parent table
  for(auto id : this->data) {
    dis[id.first] = 100000.0; // infinte for each node to loc1
    parent[id.first] = ""; // non-parent conect
    visited[id.first] = false;
  }
  visited[loc1_id] = true;
  // Intialize the first node to neighbors distances
  for(std::string& neighbor_id : this->data[loc1_id].neighbors) {
    double weight = CalculateDistance(this->data[loc1_id], this->data[neighbor_id]);
    dis[neighbor_id] = weight;
    parent[neighbor_id] = loc1_id;
  }
  // Start Greedy approach
  for(int i = 0; i < this->data.size(); ++i) {
    std::string curr_id = findMinDistanceButNotVisited(dis, visited);
    visited[curr_id] = true;
    for(std::string neighbor : this->data[curr_id].neighbors) {
      double next_neighbor = CalculateDistance(this->data[curr_id], this->data[neighbor]);
      if(!visited[neighbor] && dis[curr_id] + next_neighbor < dis[neighbor]) {
        dis[neighbor] = dis[curr_id] + next_neighbor;
        parent[neighbor] = curr_id;
      }
    }
  }
  // Do Backtracking to find the path
  printPath(loc2_id, parent, path);
  std::reverse(path.begin(), path.end());
  if(path.size() == 0) return {};
  path.push_back(loc2_id);
  return path;
}

/**
 * CalculateShortestPathBellman
 * Using Bellman Ford Alogrithms
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPathBellman(std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::unordered_map<std::string, bool> visited;
  std::unordered_map<std::string,double> distance;
  std::unordered_map<std::string,std::string> parent;
  std::vector<std::pair<double, std::pair<std::string,std::string>>> each_dis;
  for(auto id : this->data) {
    distance[id.first] = 100000.0;
    parent[id.first] = "";
    visited[id.first] = false;
    for(auto n : id.second.neighbors) {
      double dis = CalculateDistance(data[id.first],data[n]);
      each_dis.push_back({dis, {n, id.first}});
    }
  }
  if(this->name_id.find(location1_name) == this->name_id.end()) return {};
  else distance[name_id[location1_name]] = 0.0;
  if(this->name_id.find(location2_name) == this->name_id.end()) return {};
  std::string loc1_id = this->name_id[location1_name];
  std::string loc2_id = this->name_id[location2_name];

  for (int i = 1 ; i <= data.size()-1 ; ++i){
    bool flag = false;
    
    for (int j =0; j < each_dis.size(); ++j){
      std::string u = each_dis[j].second.first;
      std::string v = each_dis[j].second.second;
      double w = each_dis[j].first;
      if (distance[u] != 100000.0 && distance[u]+ w <distance[v]){
        distance[v] = distance[u] + w;
        parent[v] = u;
        flag = true;
      }
    }
    if(!flag && i != 1) break;
  }
  printPath(loc2_id,parent,path);
  std::reverse(path.begin(),path.end());
  if(path.size() == 0) return {};
  path.push_back(loc2_id);
  return path;
}

std::string TrojanMap::findMinDistanceButNotVisited(std::unordered_map<std::string, double>& d, std::unordered_map<std::string, bool>& visited) {
  std::string nextShortestNode;
  double minWeight = 100001.0;
  for(auto it : d) {
    if(!visited[it.first] && it.second < minWeight) {
      nextShortestNode = it.first;
      minWeight = it.second;
    }
  }
  return nextShortestNode;
}

void TrojanMap::printPath(std::string target, std::unordered_map<std::string, std::string>& parent, std::vector<std::string>& path) {
  if(parent[target] == "") return;
  path.push_back(parent[target]);
  printPath(parent[target], parent, path);
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(std::vector<std::string> &location_ids) {
  // Try all combination
  std::vector<std::vector<std::string>> process;
  std::vector<std::string> best;
  backtracking(process, location_ids, 0);
  double shortest = 100000.0;
  for(int i = 0; i < process.size(); ++i) {
    double temp = CalculatePathLength(process[i]);
    if(temp < shortest) {
      shortest = temp;
      best = process[i];
    }
  }
  process.push_back(best);
  std::pair<double, std::vector<std::vector<std::string>>> results = {shortest, process};
  return results;
} 

void TrojanMap::backtracking(std::vector<std::vector<std::string>>& ans, std::vector<std::string>& location_ids, int index) {
  if(index >= location_ids.size()) {
    std::vector<std::string> temp = location_ids;
    temp.push_back(location_ids[0]);
    ans.push_back(temp);
    return;
  }
  for(int i = index; i<location_ids.size(); ++i) {
      swap(location_ids[index], location_ids[i]);
      backtracking(ans, location_ids, index + 1);
      swap(location_ids[index], location_ids[i]);
  }
}


/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojanBruteForceImprovement(std::vector<std::string> &location_ids) {
  // Try all combination
  std::vector<std::vector<std::string>> process;
  std::vector<std::string> temp;
  std::unordered_set<std::string> choosen;
  double min_cost = 100000;
  backtracking2(process, location_ids, temp, 0, 0.0, min_cost);
  double shortest = 100000.0;
  for(int i = 0; i < process.size(); ++i) {
    double dis = CalculatePathLength(process[i]);
    if(dis < shortest) {
      shortest = dis;
      temp = process[i];
    }
  }
  process.push_back(temp);
  std::pair<double, std::vector<std::vector<std::string>>> results = {shortest, process};
  return results;
} 

double TrojanMap::backtracking2(std::vector<std::vector<std::string>>& ans, std::vector<std::string>& location_ids,
                    std::vector<std::string> temp, int curr_index, double curr_cost, double& min_cost) {
  double result = 100000.0;
  temp.push_back(location_ids[curr_index]);
  if(temp.size() == location_ids.size()) {
    temp.push_back(location_ids[0]);
    ans.push_back(temp);
    // return curr_cost + this->CalculateDistance(data[location_ids[curr_index]], data[loc]);
    return 0;
  }
  
  for(int i = 0; i < location_ids.size(); ++i) {
    if(i != curr_index && std::find(begin(temp), end(temp), location_ids[i]) == temp.end()) {
      if(curr_cost < min_cost) {
        result = std::min(result, backtracking2(ans, location_ids, temp, i, 
                        curr_cost + CalculateDistance(data[location_ids[curr_index]], data[location_ids[i]]), min_cost));
      }
    }
  }
  return result;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan2_Opt(std::vector<std::string> &location_ids) {
  // Save all path
  std::vector<std::vector<std::string>> process;
  // break condition: if no more improve it will break out the while loop
  int improve = 0;
  // Target
  std::vector<std::string> path(begin(location_ids), end(location_ids));
  path.push_back(location_ids[0]);
  // Current Distance
  double curr_dis = CalculatePathLength(path);
  int n = path.size();
  while(improve < 2 * n) {
    for(int i = 1; i < n - 2; ++i) {
      for(int j = i + 1; j < n -1 ; ++j) {
        std::vector<std::string> new_path = twoNodeSwap(i, j, path);
        double new_dis = CalculatePathLength(new_path);
        if(new_dis < curr_dis) {
          path = new_path;
          curr_dis = new_dis;
          improve = 0;
          process.push_back(path);
          //break out this iteration
          i = n;
          j = n;
        }
      }
    }
    ++ improve;
  }
  return {curr_dis, process};
}

std::vector<std::string> TrojanMap::twoNodeSwap(int i, int j, std::vector<std::string>& path) {
  int n = path.size();
  std::vector<std::string> new_path(n);
  for(int c = 0; c < i; ++c) new_path[c] = path[c];
  int dis = 0;
  for(int c = i; c <= j; ++c) {
    new_path[c] = path[j - dis];
    ++ dis;
  };
  for(int c = j + 1; c < n; ++c) new_path[c] = path[c];
  return new_path;
}
