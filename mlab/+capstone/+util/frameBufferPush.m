function buf = frameBufferPush(buf, rt)
    buf.idx = mod(buf.idx, buf.maxN) + 1;
    buf.data(buf.idx) = rt;
    buf.valid(buf.idx) = true;
end
