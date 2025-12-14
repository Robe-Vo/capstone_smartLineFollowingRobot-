function buf = frameBufferInit(maxN)
    buf.maxN = maxN;
    buf.idx = 0;
    buf.data = repmat(capstone.types.makeRuntimeFrame(), maxN, 1);
    buf.valid = false(maxN,1);
end
