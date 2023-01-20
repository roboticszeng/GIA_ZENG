function [int, dec] = convert_float_to_intdec(f, bit)

int = floor(f);

dec = round((f - floor(f)) * 2^bit);

if bit == 8
    int = int * 256 + dec;
end

