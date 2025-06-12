# Error Class

The `Parser` class is responsible for reading and parsing the simulation environment from a file. It takes a file name (passed from the command line) and provides a method to extract and return its content as a list of strings.


```c++
class Parser {
    public:
        
        Parser(int ac, const std::string &filename);
        
        std::vector<std::string> parseFile();
        
        ~Parser() = default;    
    
    private:
        std::string _filename;
};
```

---

<div style="max-width:500px; margin:40px auto; border-radius:16px; overflow:hidden; box-shadow:0 10px 20px rgba(0,0,0,0.15); transition:transform 0.3s ease;" onmouseover="this.style.transform='scale(1.02)'" onmouseout="this.style.transform='scale(1)'">
  <img src="/images/work_session_pole_it/together2.jpeg" alt="Patrice DAGBE" style="width:100%; height:auto; display:block;">
</div>

## Attributes
```c++
// Stores the path of the file to parse
std::string _filename;
```

## Methods

### Constructor
```c++
Parser::Parser(int ac, const std::string &filename);
```
  - Initializes the parser with a filename.
  - The ac parameter represents the argument count passed from the command line
  - Stores the filename to be used when calling parseFile().

### Destructor
```c++
Parser::~Parser() = default;
```
  - Uses the default destructor, as no manual memory management is needed.

### Method
```c++
std::vector<std::string> Parser::parseFile()
{
    std::vector<std::string> map;
    //Opens the file given by _filename.
    std::ifstream file(_filename);

    //Throws an Error if the file can't be opened.
    if (!file.is_open()) {
        throw Error("Error : impossible to open the map's file");
        return map;
    }

    //Reads each non-empty line and adds it to a vector.
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty())
            map.push_back(line);
    }

    file.close();
    //Returns the vector of strings (the map).
    return map;
}
```
  - Processes the file passed on the command line by loading and reading it.
  - Returns a double array of `std::string` representing the map
