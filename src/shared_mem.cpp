#include "shared_mem.h"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/time.h>

SharedMem::SharedMem(const std::string &shm_name, uint32_t shm_size)
{
    shm_fd = open(shm_name.c_str(), O_RDWR | O_CREAT, 0666);
    if(shm_fd < 0)
    {
        printf("Failed to create the file for shared memory. (%d)\n", errno);
        return;
    }
    lseek(shm_fd, shm_size-1, SEEK_SET);
    write(shm_fd, "", 1);
    lseek(shm_fd, 0, SEEK_SET);
    shm_buf = (uint32_t *)mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if((void *)shm_buf == MAP_FAILED)
    {
        printf("Failed to create the shared memory. (%d)\n", errno);
        close(shm_fd);
        return;
    }
}

SharedMem::~SharedMem()
{
    if(shm_buf != MAP_FAILED)
        munmap(shm_buf, shm_size);
    if(shm_fd >= 0)
        close(shm_fd);
}

bool SharedMem::write_data(uint32_t *data, std::size_t size){
    if(!shm_buf || shm_buf == MAP_FAILED){
        return false;
    }
    if(-1 == flock(shm_fd, LOCK_EX))
        return false;

    std::memcpy(shm_buf, data, size);

    flock(shm_fd, LOCK_UN);

    return true; 
}