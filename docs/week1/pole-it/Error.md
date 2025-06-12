# Error Class

The `Error` class inherits from the standard `std::exception` class and is intended to handle all errors occurring in the program. Specific error messages are then displayed.

```c++
class Error : public std::exception {
    public:
        Error(std::string msg): _msg(msg) {};

        ~Error() = default;

        const char *what() const noexcept override {return _msg.c_str(); };

    private:
        std::string _msg;
};
```

## Attributes
```c++
// Stores the error message to be displayed when the exception is thrown
std::string _msg;
```

## Methods

### Constructor
```c++
Error::Error(std::string msg): _msg(msg) {};
```
  - Initializes the error message that will be returned when the `what()` function is called.

### Destructor
```c++
~Error() = default;
```
  - Uses the default destructor (no special cleanup needed).

### Method
```c++
const char *what() const noexcept override {return _msg.c_str();};
```
  - Overrides the standard `what()` method `of std::exception` class.
  - Returns the error message as a C-style string `(const char*)`, which can be used in catch blocks or for logging.

---

<div style="max-width:500px; margin:40px auto; border-radius:16px; overflow:hidden; box-shadow:0 10px 20px rgba(0,0,0,0.15); transition:transform 0.3s ease;" onmouseover="this.style.transform='scale(1.02)'" onmouseout="this.style.transform='scale(1)'">
  <img src="/images/work_session_pole_it/maite2.jpeg" alt="Patrice DAGBE" style="width:100%; height:auto; display:block;">
</div>

```python
print("Yes i am a girl, yes i code ...!")
```
