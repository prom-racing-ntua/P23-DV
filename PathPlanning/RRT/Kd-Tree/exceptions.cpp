#include <exception>

class TreeEmptyError : public std::exception {
    public:
    char * what () {
        return "Cannot search empty tree.";
    }
};

class InvalidEntryError : public std::exception {
    public:
    char * what () {
        return "Entry is invalid. Length must be equal to the dimensions of the kd-tree.";
    }
};

class DuplicateEntryError : public std::exception {
    public:
    char * what () {
        return "Entry already exists inside the tree.";
    }
};