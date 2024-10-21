#ifndef TPT_BASEHELPER_H
#define TPT_BASEHELPER_H

#include <iostream>
//#include <filesystem>
#include <string.h>
#include <sys/stat.h>
//#include <vector>
//#include <map>
//#include <string.h>
//#include <cmath>
//#include <fstream>
//#include <cassert>
//#include <iomanip>
//#include <sstream>
//#include <regex>

namespace tpt {

class BaseHelper
{
public:
    std::string htype;

    BaseHelper();
    ~BaseHelper();

    // template and friend func
    friend std::ostream& operator<<(std::ostream &s, const BaseHelper &base){
        s << "(" << base.htype << ")";
        return s;
    }

    static bool doesDirExist(std::string _path);
    static bool doesFileExist(std::string _file);
    static std::string getDirectory (const std::string& path);
    static std::string getFileName(const std::string& filePath, bool withExtension = true, char seperator = '/');
    static std::string replaceExtension(const std::string& filePath, std::string newExt);

//    // type conversion support
//    template<typename T> static bool inArray(std::vector<T> arr, T ele);
//    template<typename T> static std::vector<std::string> cvt2str(std::vector<T> arr);
//    template<typename T> static std::vector<std::string> cvt2str(const T* arr, int length);


//    // normal support func: os.path
//    static std::vector<std::string> splitPath(std::string filePath);                   // os.path.split -> return dirname, fname, fname_noext, ext
//    static std::vector<std::string> listDir(std::string dir, bool retPath=false);      // os.listdir()
//    static std::string fullName(std::string path);
//    static std::string baseName(std::string path);
//    static std::string ext(std::string path);
//    static std::string parDir(std::string path);
//    static std::string parPath(std::string path);
//    static std::string withName(std::string path, std::string name);
//    static void makedirs(std::string dir, bool exist_ok=true);                          // os.makedirs
//    static std::string joinPath(std::vector<std::string> modules);                     // os.path.join
//    static std::vector<std::string> findExt(std::string dir, std::string ext);
//    static bool isDirectory(std::string path);
//    static bool isFile(std::string path);
//    static std::string incrementPath(std::string path, std::string sep = "_");    // sep.join(str_arr)

//    // normal support func: std::string
//    static bool startswith(std::string const& str, std::string const& prefix);          // string.startswith
//    static bool endswith(std::string const& str, std::string const& suffix);            // string.endswith
//    static std::string duplicate(std::string const& str, int times);                    // string.duplicate
//    static std::string joinArray(std::vector<std::string> arr, std::string sep=" ");    // sep.join(str_arr)
//    template<typename T> static std::string joinArray(std::vector<T> arr, std::string sep=" ");    // sep.join(str_arr)
//    static std::vector<std::string> split(std::string str, bool strip=false, std::string sep=" ");
//    static std::vector<std::string> split(std::string str, std::vector<std::string> delimiters, bool strip=false, bool getDelimiter=false);
//    static std::string splitSelect(std::string str, std::string sep=" ", int idx=0);
//    static std::string strip(std::string str);

//    // normal support func: char
//    static bool isPositiveInteger(std::string str);
//    static bool isDigit(char c);
//    static bool isLowerLetter(char c);
//    static bool isSpace(char c);

//    // primitive type logic support
//    template<typename T> static bool inRange(T value, T min, T max);
//    template<typename T> static T round(T val, int decimal=2);
//    template<typename T> static std::string roundStr(T val, int decimal=2);
//    template<typename T> static std::vector<std::string> roundArr(std::vector<T> arr, int decimal=2);
//    template<typename T> static T clamp(const T value, const T lower, const T upper);

};

} // namespace tpt

#endif // TPT_BASEHELPER_H
