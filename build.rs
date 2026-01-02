fn main() {
    // Set LD_LIBRARY_PATH for esp-clang to find libxml2.so.2 and other dependencies
    std::env::set_var("LD_LIBRARY_PATH", "/usr/lib64:/usr/lib");
    
    embuild::espidf::sysenv::output();
}
