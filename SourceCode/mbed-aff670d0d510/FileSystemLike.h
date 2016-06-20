/* mbed Microcontroller Library - FileSystemLike
 * Copyright (c) 2008-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_FILESYSTEMLIKE_H
#define MBED_FILESYSTEMLIKE_H

#ifdef __ARMCC_VERSION
# define O_RDONLY 0
# define O_WRONLY 1
# define O_RDWR   2
# define O_CREAT  0x0200
# define O_TRUNC  0x0400
# define O_APPEND 0x0008
typedef int mode_t;
#else
# include <sys/fcntl.h>
#endif
#include "Base.h"
#include "FileHandle.h"
#include "DirHandle.h"

namespace mbed {

/** A filesystem-like object is one that can be used to open files
 *  though it by fopen("/name/filename", mode)
 *
 *  Implementations must define at least open (the default definitions
 *  of the rest of the functions just return error values).
 */
class FileSystemLike : public Base {

 public:

    /** FileSystemLike constructor
     *
     *  @param name The name to use for the filesystem.
     */
    FileSystemLike(const char *name) : Base(name) {}

    /** Opens a file from the filesystem
     *
     *  @param filename The name of the file to open.
     *  @param flags One of O_RDONLY, O_WRONLY, or O_RDWR, OR'd with
     *    zero or more of O_CREAT, O_TRUNC, or O_APPEND.
     *
     *  @returns
     *    A pointer to a FileHandle object representing the
     *    file on success, or NULL on failure.
     */
    virtual FileHandle *open(const char *filename, int flags) = 0;

    /** Remove a file from the filesystem.
     *
     *  @param filename the name of the file to remove.
     *  @param returns 0 on success, -1 on failure.
     */
    virtual int remove(const char *filename) { return -1; };

    /** Rename a file in the filesystem.
     *
     *  @param oldname the name of the file to rename.
     *  @param newname the name to rename it to.
     *
     *  @returns
     *    0 on success,
     *   -1 on failure.
     */
    virtual int rename(const char *oldname, const char *newname) { return -1; };

    /** Opens a directory in the filesystem and returns a DirHandle
     *  representing the directory stream.
     *
     *  @param name The name of the directory to open.
     *
     *  @returns
     *    A DirHandle representing the directory stream, or
     *    NULL on failure.
     */
    virtual DirHandle *opendir(const char *name) { return NULL; };

    /** Creates a directory in the filesystem.
     *
     *  @param name The name of the directory to create.
     *  @param mode The permissions to create the directory with.
     * 
     *  @returns
     *    0 on success,
     *   -1 on failure.
     */
    virtual int mkdir(const char *name, mode_t mode) { return -1; }

    // TODO other filesystem functions (mkdir, rm, rn, ls etc)
    
};

} // namespace mbed

#endif
