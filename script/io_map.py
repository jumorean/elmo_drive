#!/usr/bin/python3
import json
include_dir = "/home/cda/code/examples/standing/include"
src_dir = "/home/cda/code/examples/standing/src"

h_file_name = include_dir + "/io.h"
cpp_file_name = src_dir + "/io.cpp"

config_dir = "/home/cda/code/examples/standing/config/io"

cpp_file = open(cpp_file_name, mode="w")
h_file = open(h_file_name, mode="w+")

end_line = "\n"
indent = "\t"

total_sdo_write_nums = 0


def cpp_file_head():
    cpp_file.write("#include \"io.h\"\n") #
    cpp_file.write("static int returnValue = 0;\n")
    cpp_file.write("static int config_check(int wkc)\n")
    cpp_file.write("{\n\tint success = 0;\n\tif(wkc==1)\n\t{\n\t\treturnValue ++;\n")
    cpp_file.write("\t\tprintf(\"config success, returnValue = %d\\n\", returnValue);\n")
    cpp_file.write("\t\tsuccess = 1;\n\t}\n"
                   "\telse\n\t\t{\n\t\tstd::cerr << \"config error, returnValue = \" "
                   "<< returnValue << \"\\n\";\n")
    cpp_file.write("\t\tsuccess = 0;\n\t}\n")
    cpp_file.write("\treturn success;\n")
    cpp_file.write("}\n\n\n")
    cpp_file.write("int io_config()\n{\n")


def subs_nums_clear(index):
    cpp_file.write(indent + "sdo_data8 = " + hex(0) + "U;\n")
    cpp_file.write(indent + "config_check(ec_SDOwrite(cnt, ")
    cpp_file.write(index + "U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));\n")
    cpp_file.write("\n")
    global total_sdo_write_nums
    total_sdo_write_nums += 1


def subs_nums_set(index, nums):
    cpp_file.write("\n")
    cpp_file.write(indent + "sdo_data8 = " + hex(nums) + "U;\n")
    cpp_file.write(indent + "config_check(ec_SDOwrite(cnt, ")
    cpp_file.write(index + "U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));\n")
    global total_sdo_write_nums
    total_sdo_write_nums += 1


def subs_write32(target_index, count, sdo_data32):
    cpp_file.write(indent + "sdo_data32 = " + sdo_data32 + ";\n")
    cpp_file.write(indent + "config_check(ec_SDOwrite(cnt, ")
    cpp_file.write(target_index + "U, " + str(count) + "U, FALSE, 4, p32, EC_TIMEOUTRXM));\n")
    global total_sdo_write_nums
    total_sdo_write_nums += 1


def subs_write16(index, cnt, sdo_data16):
    cpp_file.write(indent + "sdo_data16 = " + sdo_data16 + "U;\n")
    cpp_file.write(indent + "config_check(ec_SDOwrite(cnt, ")
    cpp_file.write(index + "U, " + str(cnt) + "U, FALSE, 2, p16, EC_TIMEOUTRXM));\n")
    global total_sdo_write_nums
    total_sdo_write_nums += 1


def start_comment(index):
    cpp_file.write(indent + "/* " + index + " mapping start */" + end_line)


def end_comment(index):
    cpp_file.write(indent + "/* " + index + " mapping end */" + end_line)


def write_function_head():
    lines = [
        indent + "uint32 sdo_data32=0;" + end_line,
        indent + "uint16 sdo_data16=0;" + end_line,
        indent + "uint8 sdo_data8=0;" + end_line,
        indent + "void * p32 = (void *)(&sdo_data32);" + end_line,
        indent + "void * p16 = (void *)(&sdo_data16);" + end_line,
        indent + "void * p8 = (void *)(&sdo_data8);" + end_line,
        indent + "returnValue = 0;" + end_line,
        indent + "for(int cnt = 1; cnt <= ec_slavecount ; cnt++)" + end_line,
        indent + "{" + end_line
    ]
    cpp_file.writelines(lines)


def pdo2sdo_data32(pdo):
    data_type = pdo["data type"]
    if "u" in data_type:
        bit_size_str_dec = data_type.replace("uint", "")
    else:
        bit_size_str_dec = data_type.replace("int", "")
    bit_size_str_hex = hex(int(bit_size_str_dec)).replace("0x", "", 1)
    if len(bit_size_str_hex) == 1:
        bit_size_str_hex = "0" + bit_size_str_hex
    if "subIndex" in pdo:
        sub_index_str = pdo["subIndex"].replace("0x", "", 1)
    else:
        sub_index_str = "00"
    index_str = pdo["index"]
    sdo_data32_str = index_str + sub_index_str + bit_size_str_hex + "U"
    assert len(sdo_data32_str) == 11
    return sdo_data32_str


def pdo_map(target_index):
    cpp_file.write(end_line)
    start_comment(target_index)
    with open(config_dir + "/" + target_index + ".json", "r") as pdo_group_config_file:
        pdo_group_config = json.load(pdo_group_config_file)
        pdo_name_array = list(pdo_group_config)
        subs_nums_clear(target_index)
        cnt_local = 0
        for pdo_name in pdo_name_array:
            cnt_local += 1
            pdo = pdo_group_config[pdo_name]
            h_file.write("\t" + pdo["data type"] + "\t" + pdo_name + ";\t\t\t\t//" + pdo["index"] + "\n")
            subs_write32(target_index, cnt_local, pdo2sdo_data32(pdo))
        subs_nums_set(target_index, len(pdo_name_array))
    end_comment(target_index)


def h_file_head():
    h_file.write("#ifndef IO_H\n")
    h_file.write("#define IO_H\n\n\n")
    h_file.write("#include <iostream>\n")
    h_file.write("#include <ethercat.h>\n")


def struct_head():
    h_file.write("\n\ntypedef struct PACKED\n")
    h_file.write("{\n")


def struct_tail(io_type):
    h_file.write("}" + io_type.capitalize() + "Data_t;\n")


def pdo_group_map(io_config, io_type):
    target_index_array = io_config[io_type]["items"]
    struct_head()
    for target_index in target_index_array:
        pdo_map(target_index)
    struct_tail(io_type)


def io_manage(io_config, io_type):
    target_index = io_config[io_type]["index"]
    pdo_group_array = io_config[io_type]["items"]
    # /*1c13h mapping start*/
    start_comment(target_index)
    subs_nums_clear(target_index)
    io_count = 0
    for pdo_group in pdo_group_array:
        io_count += 1
        subs_write16(target_index, io_count, pdo_group)
    subs_nums_set(target_index, len(pdo_group_array))
    end_comment(target_index)
    # /*1c13h mapping end*/


def function_decl():
    h_file.write("\n\nint io_config();\n\n\n")


def cpp_file_tail():
    cpp_file.write("\t}" + end_line)
    cpp_file.write("\tint total_sdo_write_nums = " + str(total_sdo_write_nums) + ";" + end_line)
    cpp_file.write("\tif(total_sdo_write_nums * ec_slavecount == returnValue)" + end_line)
    cpp_file.write("\t\treturn 0;" + end_line)
    cpp_file.write("\telse return 1;" + end_line)
    cpp_file.write("}\n")


def h_file_tail():
    h_file.write("#endif /* IO_H */\n")



def main():
    h_file_head()
    cpp_file_head()
    with open(config_dir + "/io.json", "r") as config_file:
        global indent
        config = json.load(config_file)
        write_function_head()
        indent += "\t"
        pdo_group_map(config, "input")
        pdo_group_map(config, "output")
        io_manage(config, "input")
        io_manage(config, "output")
    function_decl()
    cpp_file_tail()
    h_file_tail()
    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=")
    print("success")
    global total_sdo_write_nums
    print("total_sdo_write_nums = ", total_sdo_write_nums)


if __name__ == '__main__':
    main()

