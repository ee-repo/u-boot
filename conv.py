import sys, os

if __name__ == '__main__':
    args = sys.argv
    if len(args) < 3:
        print_usage()
        exit(-1)
    if not os.access(args[1], os.F_OK):
        print 'input file %s is not exist' % (args[1])
        exit(-1)
    f_in = os.open(args[1], os.O_RDONLY)
    f_out = os.open(args[2], os.O_RDWR | os.O_CREAT)
    os.ftruncate(f_out, 0x80000)

    for blk in range(0, 4):
        for pos in range(0, os.stat(args[1]).st_size/0x400):
            os.lseek(f_in, 0x400*pos, os.SEEK_SET)
            temp = os.read(f_in, 0x800)
            os.lseek(f_out, 0x20000*blk + 0x800*pos, os.SEEK_SET)
            os.write(f_out, temp)
    os.close(f_in)
    os.close(f_out)
