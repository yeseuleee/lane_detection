#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <exception>

#define MAX_SLOPE 9999

using std::vector;


struct Pos{
  Pos(): x(0), y(0) {}
  Pos(int x, int y) : x(x), y(y) {}
  Pos(double x, double y): x(x), y(y) {}
  double x;
  double y;
};

struct Line{
  void parseLine(std::string filename);
  double slope;
  double intercept;
};

std::istream& operator>>(std::istream& is, Pos& pos){
  char ch;
  is >> pos.x >> ch >> pos.y >> ch;
  return is;
}

std::istream& operator>>(std::istream& is, Line& line){
  char ch;
  is >> line.slope >> ch >> line.intercept >> ch;
  return is;
}

void parsePixel(std::string filename, vector<vector<Pos> >& posVec, Pos& center){

  std::ifstream in(filename);
  std::string err= "file open error! in parsePixel... : " + filename;
  if(in.fail()) throw std::runtime_error(err.c_str());
  //initialize center
  char ch;
  std::string str;
  in >> center.x >> ch >> center.y;
  in >> str;//flush newline

  //initialize pixel
  while(!in.eof()){
    posVec.emplace_back(0); // append

    std::stringstream ss(str);
    while(!ss.eof()){
      Pos p;
      ss >> p;
      posVec.back().push_back(p);
    }
    in >> str;
  }
}

void parseLine(std::string filename, vector<Line> & hlines, vector<Line>& vlines){

  std::ifstream in(filename);
  if(in.fail()) throw std::runtime_error("file open error! in parseLine...");
  std::string str;

  //parse hlines
  in >> str;
  std::stringstream ss(str);
  while(!ss.eof()){
    Line line;
    ss >> line;
    hlines.push_back(line);
  }

  //parse vlines
  in >> str;
  ss.clear(); ss.str(str);
  while(!ss.eof()){
    Line line;
    ss >> line;
    vlines.push_back(line);
  }
}
