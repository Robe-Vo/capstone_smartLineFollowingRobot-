function out = frameBufferGetLastK(buf, k)
    k = min(k, buf.maxN);
    ids = mod((buf.idx - k + 1 : buf.idx) - 1, buf.maxN) + 1;
    ids = ids(buf.valid(ids));
    out = buf.data(ids);
end
