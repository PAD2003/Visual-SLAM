#include "basehelper.h"

namespace tpt {

BaseHelper::BaseHelper()
{

}

BaseHelper::~BaseHelper()
{

}

bool BaseHelper::doesDirExist(std::string _path)
{
    // Structure buffer which would store the metadata
    struct stat sb;
    if (stat(_path.c_str(), &sb) == 0)
        return true;
    else
        return false;
}

bool BaseHelper::doesFileExist(std::string _file)
{
    // Structure buffer which would store the metadata
    struct stat sb;

    // Calls the function with path as argument
    // If the file/directory exists at the path returns 0
    // If block executes if path exists
    if (stat(_file.c_str(), &sb) == 0 && !(sb.st_mode & S_IFDIR))
        return true;
    else
        return false;
}

//std::string getDirectory (const std::string& path)
//{
//    std::experimental::filesystem::path p(path);
//    return(p.parent_path().string());
//}

std::string BaseHelper::getFileName(const std::string& filePath, bool withExtension, char seperator)
{
    // Get last dot position
    std::size_t dotPos = filePath.rfind('.');
    std::size_t sepPos = filePath.rfind(seperator);
    if (sepPos != std::string::npos)
    {
        return filePath.substr(sepPos + 1, filePath.size() - (withExtension || dotPos != std::string::npos ? 1 : dotPos) );
    }
    return "";
}

std::string BaseHelper::replaceExtension(const std::string &filePath, std::string newExt)
{
    // Get last dot position
    std::size_t dotPos = filePath.rfind('.');
    if (dotPos != std::string::npos)
    {
        return filePath.substr(0, dotPos) + newExt;
    }

    return filePath;
}

///* Type Conversion supporter */
//template<typename T>
//bool BaseHelper::inArray(std::vector<T> arr, T ele){
//    auto it = std::find(arr.begin(), arr.end(), ele);
//    return (it != arr.end());
//}

//template<typename T>
//std::vector<std::string> BaseHelper::cvt2str(std::vector<T> arr){
//    std::vector<std::string> res;
//    for (T ele : arr)
//        res.push_back(std::to_string(ele));
//    return res;
//}

//template<typename T>
//std::vector<std::string> BaseHelper::cvt2str(const T* arr, int length){
//    std::vector<std::string> res;
//    for (int i = 0; i < length; i++)
//        res.push_back(std::to_string(arr[i]));
//    return res;
//}

///* Path supporter */
//std::vector<std::string> BaseHelper::splitPath(std::string filePath){
//    std::filesystem::path file(filePath);
//    std::filesystem::path head = file.parent_path();
//    std::filesystem::path tail = file.filename();
//    std::filesystem::path filename = tail.stem();
//    std::filesystem::path ext = tail.extension();
//    return (std::vector<std::string>){
//        head.string(), tail.string(),
//        filename.string(), ext.string()
//    };
//}

//std::vector<std::string> BaseHelper::listDir(std::string dirPath, bool retPath){
//    std::vector<std::string> res;
//    const std::filesystem::path dir(dirPath);
////    ILOG_CEVENT_ERROR(!std::filesystem::is_directory(dir), dexception::FileNotFoundException, "No such directory {}", dirPath);
//    for (const std::filesystem::directory_entry& x : std::filesystem::directory_iterator(dir)){
//        const std::filesystem::path xpath = x.path();
//        if (retPath)
//            res.push_back(xpath.string());
//        else
//            res.push_back(xpath.filename().string());
//    }
//    return res;
//}

//std::string BaseHelper::fullName(std::string path){
//    std::filesystem::path file(path);
//    return file.filename();
//}

//std::string BaseHelper::baseName(std::string path){
//    std::filesystem::path file(path);
//    return file.filename().stem();
//}

//std::string BaseHelper::ext(std::string path){
//    std::filesystem::path file(path);
//    return file.filename().extension();
//}

//std::string BaseHelper::parDir(std::string path){
//    std::filesystem::path file(path);
//    return file.parent_path().stem();
//}

//std::string BaseHelper::parPath(std::string path){
//    std::filesystem::path file(path);
//    return file.parent_path();
//}

//std::string BaseHelper::withName(std::string path, std::string name){
//    std::filesystem::path file(path);
//    file.replace_filename(name);
//    return file.string();
//}


//bool BaseHelper::startswith(std::string const& str, std::string const& prefix){
//    return (str.compare(0, prefix.size(), prefix) == 0);
//}

//bool BaseHelper::endswith(std::string const& str, std::string const& suffix){
//    if (str.length() < suffix.length())
//        return false;
//    return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
//}

//void BaseHelper::makedirs(std::string dir, bool exist_ok){
//    std::filesystem::path dirPath = dir;
//    bool success = std::filesystem::create_directories(dirPath);
//    ILOG_CEVENT_ERROR((!exist_ok && !success), dexception::FileNotFoundException, "The directory {} has already existed.", dir);
//}

//std::string BaseHelper::joinPath(std::vector<std::string> modules){
//    if (modules.empty()) return "";
//    std::filesystem::path p = "";
//    for (auto module : modules){
//        std::filesystem::path m = module;
//        p = p / m;
//    }
//    return p.string();
//}

//std::vector<std::string> BaseHelper::findExt(std::string dir, std::string ext){
//    std::regex pattern(".*" + ext + "$");
//    std::vector<std::string> res;

//    for (const auto& entry : std::filesystem::directory_iterator(dir))
//        if (std::filesystem::is_regular_file(entry.path()) && std::regex_match(entry.path().filename().string(), pattern))
//            res.push_back(entry.path().filename().stem().string());

//    return res;
//}

//bool BaseHelper::isDirectory(std::string path){
//    return (std::filesystem::exists(path) && std::filesystem::is_directory(path));
//}

//bool BaseHelper::isFile(std::string path){
//    return (std::filesystem::exists(path) && std::filesystem::is_regular_file(path));
//}

//std::string BaseHelper::incrementPath(std::string path, std::string sep){
//    std::string baseName = detector::helper::BaseHelper::baseName(path);
//    std::string parPath = detector::helper::BaseHelper::parPath(path);
//    std::string pattern = detector::helper::BaseHelper::splitSelect(baseName, sep, -1);
//    std::smatch matches;
//    std::vector<std::string> files = detector::helper::BaseHelper::listDir(parPath, true);
//    std::vector<int> matchIds{0};

//    for (auto file : files){
//        std::regex_search(file, matches, std::regex(pattern));
//        try {
//            matchIds.push_back(std::stoi(matches[1]));
//        } catch (const std::exception& e){
//            continue;
//        }
//    }
//    std::sort(matchIds.rbegin(), matchIds.rend());
//    return path.replace(path.find(pattern), pattern.length(), std::to_string(matchIds[0] + 1));
//}

///* String supporter */
//std::string BaseHelper::duplicate(std::string const& str, int times){
//    std::string res;
//    for (int i = 0; i < times; ++i)
//        res += str;
//    return res;
//}

//std::string BaseHelper::joinArray(std::vector<std::string> arr, std::string sep){
//    std::string res = "";
//    for (size_t i = 0; i < arr.size(); i++){
//        res += arr[i];
//        if (i < arr.size() - 1)
//            res += sep;
//    }
//    return res;
//}

//template <typename T>
//std::string BaseHelper::joinArray(std::vector<T> arr, std::string sep){
//    std::vector<std::string> strArr = cvt2str<T>(arr);
//    return joinArray(strArr, sep);
//}

//std::vector<std::string> BaseHelper::split(std::string str, bool strip, std::string sep){
//    std::vector<std::string> sstr;
//    int start, end = -1*sep.size();
//    do {
//        start = end + sep.size();
//        end = str.find(sep, start);
//        sstr.push_back(str.substr(start, end - start));
//    } while (end != -1);
//    if (strip) for (size_t i = 0; i < sstr.size(); i++) sstr[i] = BaseHelper::strip(sstr[i]);
//    return sstr;
//}

//std::vector<std::string> BaseHelper::split(std::string str, std::vector<std::string> delimiters, bool strip, bool getDelimiter){
//    std::vector<std::string> result;
//    size_t start = 0;
//    size_t end = 0;

//    while (start < str.size()) {
//        size_t minPos = std::string::npos;
//        std::string foundDelimiter;

//        // Find the closest delimiter
//        for (const auto& delimiter : delimiters) {
//            size_t pos = str.find(delimiter, start);
//            if (pos < minPos) {
//                minPos = pos;
//                foundDelimiter = delimiter;
//            }
//        }

//        // If no delimiter is found, break the loop
//        if (minPos == std::string::npos) {
//            break;
//        }

//        end = minPos;
//        result.push_back(str.substr(start, end - start));
//        if (getDelimiter) result.push_back(foundDelimiter);
//        start = end + foundDelimiter.size();

//        // Handle consecutive delimiters
//        if (start == end) {
//            result.push_back("");
//        }
//    }

//    // Add the last part of the string
//    result.push_back(str.substr(start));
//    if (strip) for (size_t i = 0; i < result.size(); i++) result[i] = BaseHelper::strip(result[i]);
//    return result;
//}

//std::string BaseHelper::strip(std::string str){
//    std::string result = str;
//    auto isSpace = [](unsigned char const c) -> bool {
//        return std::isspace(c);
//    };
//    // Remove leading whitespaces
//    result.erase(result.begin(), std::find_if(result.begin(), result.end(),
//                                            [&](unsigned char c) { return !isSpace(c); }));
//    // Remove trailing whitespaces
//    result.erase(std::find_if(result.rbegin(), result.rend(),
//                            [&](unsigned char c) { return !isSpace(c); }).base(), result.end());
//    return result;
//}

//std::string BaseHelper::splitSelect(std::string str, std::string sep, int idx){
//    std::vector<std::string> sstr = split(str, false, sep);
//    idx = (idx >= 0) ? idx : static_cast<int>(sstr.size() + idx);
//    return sstr[idx];
//}

//bool BaseHelper::isPositiveInteger(std::string str){
//    try {
//        size_t pos;
//        int num = std::stoi(str, &pos);
//        // Check if the entire string was consumed in the conversion and the number is positive
//        return pos == str.size() && num > 0;
//    } catch (const std::invalid_argument& e) {
//        // Not convertible to an integer
//        return false;
//    } catch (const std::out_of_range& e) {
//        // Number out of range for int
//        return false;
//    }
//}

//bool BaseHelper::isDigit(char c){
//    return ('0' <= c) && (c <= '9');
//}

//bool BaseHelper::isLowerLetter(char c){
//    return ('a' <= c) && (c <= 'z');
//}

//bool BaseHelper::isSpace(char c){
//    return std::isspace(c);
//}

///* Primitive type's logic supporter */
//template<typename T>
//bool BaseHelper::inRange(T value, T min, T max){
//    return (value >= min && value <= max);
//}

//template<typename T>
//T BaseHelper::round(T val, int decimal){
//    std::stringstream ss;
//    ss << std::fixed << std::setprecision(decimal) << val;
//    std::string roundedString = ss.str();
//    return static_cast<T>(std::stod(roundedString));
//}

//template<typename T>
//std::string BaseHelper::roundStr(T val, int decimal){
//    std::stringstream ss;
//    ss << std::fixed << std::setprecision(decimal) << val;
//    return ss.str();
//}

//template<typename T> std::vector<std::string> BaseHelper::roundArr(std::vector<T> arr, int decimal){
//    std::vector<std::string> res(arr.size());
//    for (size_t i = 0; i < arr.size(); i++) res[i] = roundStr(arr[i], decimal);
//    return res;
//}


//template <typename T>
//T BaseHelper::clamp(const T value, const T lower, const T upper) {
//    return std::max(lower, std::min(value, upper));
//}


//template bool BaseHelper::inArray<float>(std::vector<float>, float);
//template bool BaseHelper::inArray<std::string>(std::vector<std::string>, std::string);
//template std::vector<std::string> BaseHelper::cvt2str<float>(std::vector<float>);
//template std::vector<std::string> BaseHelper::cvt2str<int>(std::vector<int>);
//template std::vector<std::string> BaseHelper::cvt2str<uchar>(const uchar*, int);
//template std::vector<std::string> BaseHelper::cvt2str<float>(const float*, int);
//template std::string BaseHelper::joinArray<int>(std::vector<int>, std::string);
//template std::string BaseHelper::joinArray<float>(std::vector<float>, std::string);
//template bool BaseHelper::inRange<int>(int, int, int);
//template bool BaseHelper::inRange<size_t>(size_t, size_t, size_t);
//template float BaseHelper::round(float, int);
//template double BaseHelper::round(double, int);
//template std::string BaseHelper::roundStr(float, int);
//template std::string BaseHelper::roundStr(double, int);
//template std::vector<std::string> BaseHelper::roundArr(std::vector<float>, int);
//template std::vector<std::string> BaseHelper::roundArr(std::vector<double>, int);
//template int BaseHelper::clamp(const int, const int, const int);
//template float BaseHelper::clamp(const float, const float, const float);

} // namespace tpt
