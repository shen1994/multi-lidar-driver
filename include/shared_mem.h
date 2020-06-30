#ifndef SHARED_MEM_H_
#define SHARED_MEM_H_

#include <cstring>
#include <string>

class SharedMem
{
public:
    SharedMem(const std::string &shm_name, uint32_t shm_size);
    ~SharedMem();
    bool write_data(uint32_t *data, std::size_t size);

public:
    static constexpr const char *mod_name = "SharedMem";
    int shm_fd = -1;
    uint32_t *shm_buf = nullptr;
    uint32_t shm_size = 0;
};

#endif