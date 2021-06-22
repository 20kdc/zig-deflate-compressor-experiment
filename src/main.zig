// zig-deflate-compressor - Zig zero-allocation DEFLATE (RFC 1951) Compressor
// Written starting in 2021 by contributors
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

const std = @import("std");

// So before we continue, some specific notes about the format.
// + The world in which DEFLATE operates in is a bitstream.
//   The lowest bit of each byte is the first bit in the stream, going forwards.
//   Note however that DEFLATE can forcibly align the stream to an 8-bit boundary in some situations.
// + Most integer fields are little-endian bit-wise, such that an aligned little-endian integer field is stored 'as-is'.
// + Huffman codes are 'effectively' big-endian bit-wise.
//   Some may consider this arguably a detail of how I like to represent Huffman codes,
//    but my logic can be explained by understanding the algorithm that builds a Huffman tree from bit lengths.
//   This algorithm is based around numbers, and the resulting codes following it in this manner are in this 'big-endian' form.
//   It therefore follows that this is the canonical form for calculating the Huffman codes.
//   DEFLATE has a maximum Huffman code length of 15,
//   which gives room in a 16-bit integer for a 1-bit 'always there' prefix to indicate length when implementing the Huffman search via, say, a 65536-entry lookup table.
//   So in a decompressor, you start with 1 (the empty bitstring), and shift it left adding in new incoming bits at the right end.
//   In a compressor, you simply have a mapping from symbols to integers, with separate lengths.

// And about the compressor & fixed huffman format:
// + This compressor only uses Fixed Huffman blocks, as they're easy to write while giving some compression.
// + The literal tree uses the following lengths:
//    for lI = 0, 143 do codeLen[sym] = 8 end
//    for lI = 144, 255 do codeLen[sym] = 9 end
//    for lI = 256, 279 do codeLen[sym] = 7 end
//    for lI = 280, 287 do codeLen[sym] = 8 end
//   What this calculated to is [TABLE MOVED TO writeFxhLiteralSymbol]
// + The distance tree is 32 symbols all of length 5, effectively equivalent to a 1:1 "just write 5 bits" mapping.
//   HOWEVER, the bits are reversed as per the difference between Huffman and non-Huffman integers above.
//   Note that the last 2 symbols of the distance tree will never occur in the compressed data.
//   (This implies they're padding.)

// -- Bit Reversers --

fn reverse4(nibble: u4) u4 {
    return ((nibble & 0x8) >> 3) | ((nibble & 0x4) >> 1) | ((nibble & 0x2) << 1) | ((nibble & 0x1) << 3);
}

fn reverse8(byte: u8 ) u8 {
    return (@intCast(u8, reverse4(@truncate(u4, byte & 0xF))) << 4) | @intCast(u8, reverse4(@truncate(u4, (byte & 0xF0) >> 4)));
}

fn reverse7(asc: u7) u7 {
    return @truncate(u7, reverse8(@intCast(u8, asc)) >> 1);
}

fn reverse5(ml: u5) u5 {
    return (@intCast(u5, reverse4(@truncate(u4, ml))) << 1) | (ml >> 4);
}

fn reverse9(ml: u9) u9 {
    return (@intCast(u9, reverse8(@truncate(u8, ml))) << 1) | (ml >> 8);
}

// -- Window Management --

const DeflateWindowFind = struct {
    distance: u16,
    // NOTE: Maximum length is 258.
    length: u16
};

pub fn DeflateCompressorWindow(comptime windowSize: usize) type {
    // Past this window size, unrepresentable distances may occur,
    //  integer overflows may occur...
    // Larger than this is also not supported decoding-wise
    std.debug.assert(windowSize <= 0x8000);
    return struct {
        // Window ring
        data: [windowSize]u8,
        // Amount of the ring which is valid
        valid: usize,
        // Write position in the ring
        ptr: usize,

        const Self = @This();
        pub fn init() Self {
            return Self {
                .data = undefined,
                .valid = 0,
                .ptr = 0,
            };
        }

        fn ptrBack(self: *Self, ptr: usize) usize {
            if (ptr == 0)
                return windowSize - 1;
            return ptr - 1;
        }

        fn ptrFwd(self: *Self, ptr: usize) usize {
            if (ptr == (windowSize - 1))
                return 0;
            return ptr + 1;
        }

        // Finds as much as it can of the given data slice within the window.
        // Note that it may return a length of 0 - this means you must use a literal.
        // Note that it will NOT return a length above the length of the input data.
        // Nor will it return a length above 258 (the maximum length).
        // Note that it may return 'extended' (off the end of the window) finds, this is normal and allowed in DEFLATE.
        pub fn find(self: *Self, dataC: []const u8) DeflateWindowFind {
            var bestDistance: u16 = 0;
            var bestLength: u16 = 0;
            if (dataC.len != 0) {
                // Truncate slice to ensure that length cannot exceed maximum
                const data = if (dataC.len < 258) dataC else dataC[0..258];
                // Honestly shouldn't be possible that length is zero, but just in case.
                var ptr = self.ptrBack(self.ptr);
                // Note that distance can only ever get up to 0x8000
                var distance: u16 = 1;
                while (distance < self.valid) {
                    var subPtr = ptr;
                    var subDataAdv: u16 = 0;
                    var subLength: u16 = 0;
                    // Move forward until end
                    while (subLength < data.len) {
                        var gob: u8 = undefined;
                        if (subPtr == self.ptr) {
                            // ran out of window, start from start of written data
                            // it can be assumed we will run out of outer-loop input before we run out of inner-loop input
                            gob = data[subDataAdv];
                            subDataAdv += 1;
                        } else {
                            // inside window
                            gob = self.data[subPtr];
                            subPtr = self.ptrFwd(subPtr);
                        }
                        if (gob != data[subLength])
                            break;
                        // success, increase length
                        subLength += 1;
                    }
                    // Check length
                    if (subLength > bestLength) {
                        bestDistance = distance;
                        bestLength = subLength;
                    }
                    // Continue backwards
                    ptr = self.ptrBack(ptr);
                    distance += 1;
                }
            }
            return DeflateWindowFind {
                .distance = bestDistance,
                .length = bestLength,
            };
        }

        // Injects a byte into the window.
        // This covers situations where some known activity was performed on the DEFLATE stream.
        pub fn injectByte(self: *Self, data: u8) void {
            self.data[self.ptr] = data;
            self.ptr = self.ptrFwd(self.ptr);
            if (self.valid < windowSize)
                self.valid += 1;
        }

        // Injects a slice into the window.
        // This covers situations where some known activity was performed on the DEFLATE stream.
        pub fn inject(self: *Self, data: []const u8) void {
            // Mostly naive method for now, with some slight adjustments
            if (data.len > windowSize) {
                self.inject(data[(data.len - windowSize)..data.len]);
                return;
            }
            for (data) |b| {
                self.injectByte(b);
            }
        }

        // Resets window validity.
        // This covers situations where some unknown activity was performed on the DEFLATE stream.
        pub fn reset(self: *Self) void {
            self.valid = 0;
            self.ptr = 0;
        }
    };
}

// -- Main Body --

const DEFLATE_LENGTH_BITBASE = struct {
    // Table of extension bit counts for given length symbols, starting with 257.
    const BIT: []const u8 = &[29]u8{
        0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 2, 2, 2, 2,
        3, 3, 3, 3, 4, 4, 4, 4,
        5, 5, 5, 5, 0
    };
    // Table of base lengths for given length symbols, starting with 257.
    // First value here implies minimum length.
    const BASE: []const u16 = &[29]u16{
        3, 4, 5, 6, 7, 8, 9, 10,
        11, 13, 15, 17, 19, 23, 27, 31,
        35, 43, 51, 59, 67, 83, 99, 115,
        131, 163, 195, 227, 258
    };
};

const DEFLATE_DIST_BITBASE = struct {
    // Table of extension bit counts for given distance symbols
    const BIT: []const u8 = &[30]u8{
        0, 0, 0, 0, 1, 1, 2, 2,
        3, 3, 4, 4, 5, 5, 6, 6,
        7, 7, 8, 8, 9, 9, 10, 10,
        11, 11, 12, 12, 13, 13
    };
    // Table of base distances for given distance symbols
    const BASE: []const u16 = &[30]u16{
        1, 2, 3, 4, 5, 7, 9, 13,
        17, 25, 33, 49, 65, 97, 129, 193,
        257, 385, 513, 769, 1025, 1537, 2049, 3073,
        4097, 6145, 8193, 12289, 16385, 24577
    };
};

// Note: it's assumed that bitbases are in increasing order.
// If not, this won't work.
fn chooseBitBase(val: u16, comptime bitbase: type) usize {
    for (bitbase.BASE) |base, idx| {
        var bit = bitbase.BIT[idx];
        var extent = base + (@as(u16, 1) << @truncate(u4, bit));
        if ((val >= base) and (val < extent)) {
            return idx;
        }
    }
    std.debug.panic("failed to find {}", .{ val });
}

pub fn DeflateCompressor(comptime windowSize: usize, comptime WriterType: type) type {
    return struct {
        const Window = DeflateCompressorWindow(windowSize);
        forward_writer: BitWriterType,
        window: Window,
        const BitWriterType = std.io.BitWriter(std.builtin.Endian.Little, WriterType);
        const Self = @This();
        // NOTE: This type does not perform any dynamic allocation.
        // This operation inherently cannot fail (no writing is performed yet).
        pub fn init(src: WriterType) Self {
            return Self {
                .forward_writer = BitWriterType.init(src),
                .window = Window.init()
            };
        }

        // Internal: Write symbol, fixed-huffman-table: literal
        fn writeFxhLiteralSymbol(self: *Self, sym: u9) BitWriterType.Error!void {
            if (sym < 144) {
                //    Symbols 0, 143 inc. are 8-bit, base 48
                try self.forward_writer.writeBits(reverse8(@truncate(u8, sym + 48)), 8);
            } else if (sym < 256) {
                //    Symbols 144, 255 inc. are 9-bit, base 400
                try self.forward_writer.writeBits(reverse9(@truncate(u9, sym + 256)), 9);
            } else if (sym < 280) {
                //    Symbols 256, 279 inc. are 7-bit, base 0
                try self.forward_writer.writeBits(reverse7(@truncate(u7, sym - 256)), 7);
            } else {
                //    Symbols 280, 287 inc. are 8-bit, base 192
                try self.forward_writer.writeBits(reverse8(@truncate(u8, sym - 88)), 8);
            }
        }

        // Internal: Write symbol, fixed-huffman-table: distance
        fn writeFxhDistanceSymbol(self: *Self, sym: u5) BitWriterType.Error!void {
            try self.forward_writer.writeBits(reverse5(sym), 5);
        }

        // This opens a fixed-huffman chunk.
        // Operations not intended to be used in this mode will have terrible effects if they are used anyway.
        // Note that final must be the same between open & close calls.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn openFixedChunk(self: *Self, final: bool) BitWriterType.Error!void {
            // Final block flag.
            try self.forward_writer.writeBits(@boolToInt(final), 1);
            // Fixed Huffman block
            try self.forward_writer.writeBits(@as(u2, 1), 2);
        }

        // This closes a fixed-huffman chunk.
        // Note that final must be the same between open & close calls.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn closeFixedChunk(self: *Self, final: bool) BitWriterType.Error!void {
            try self.writeFxhLiteralSymbol(256);
            if (final) {
                try self.forward_writer.flushBits();
            }
        }

        // Writes an atomic unit of data into an active chunk.
        // This will only consume part of the data given (that part which represents this small atomic unit).
        // Returns the amount remaining, allowing you to refill it up to the window size (this allows for perfect streaming compression)
        // This is only valid in an openFixedChunk-closeFixedChunk pair.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn writeFixedUnit(self: *Self, chunk: []const u8) BitWriterType.Error![]const u8 {
            // Find...
            var find = self.window.find(chunk);
            var successChunk: []const u8 = &[0]u8{};
            // Note the use of DEFLATE_LENGTH_BITBASE here ; the minimum length is important.
            var hasDLPair: bool = find.length >= DEFLATE_LENGTH_BITBASE.BASE[0];
            if (hasDLPair) {
                // std.log.err("has DL pair", .{});
                // Verify pair is probably shorter than an equivalent literal.
                // This also does the setup for the actual writing.
                // If it is shorter, then the writing is actually done here.
                // For reference, u12 will always be correct:
                //  + max length is 258
                //  + max lit. bits is 9
                //  + therefore max size is 2322
                // Work out equivalent literal size.
                // This doesn't account for potential future finds,
                //  but that's not really likely to be a problem.
                var litBits: u12 = 0;
                for (chunk[0..find.length]) |b| {
                    if (b < 144) {
                        litBits += 8;
                    } else {
                        litBits += 9;
                    }
                }
                // Length index
                var lenIdx = chooseBitBase(find.length, DEFLATE_LENGTH_BITBASE);
                var lenSym = @intCast(u9, lenIdx) + 257;
                // Distance index
                var dstIdx = @intCast(u5, chooseBitBase(find.distance, DEFLATE_DIST_BITBASE));
                // Using those indexes, calculate bits..
                var lenSymBits: u12 = 8;
                if (lenSym < 280) {
                    lenSymBits = 7;
                }
                var dpBits: u12 = lenSymBits + DEFLATE_LENGTH_BITBASE.BIT[lenIdx] + 5 + DEFLATE_DIST_BITBASE.BIT[dstIdx];
                if (dpBits < litBits) {
                    // Length-Distance pair
                    // Length & extension...
                    try self.writeFxhLiteralSymbol(lenSym);
                    try self.forward_writer.writeBits(find.length - DEFLATE_LENGTH_BITBASE.BASE[lenIdx], DEFLATE_LENGTH_BITBASE.BIT[lenIdx]);
                    // Distance & extension...
                    try self.writeFxhDistanceSymbol(dstIdx);
                    try self.forward_writer.writeBits(find.distance - DEFLATE_DIST_BITBASE.BASE[dstIdx], DEFLATE_DIST_BITBASE.BIT[dstIdx]);
                    // Actually advance
                    successChunk = chunk[0..find.length];
                    // std.log.err("was executed, L{} - '{s}'", .{find.length, successChunk});
                } else {
                    // Not worth it
                    hasDLPair = false;
                }
            }
            if (!hasDLPair) {
                // DL pair failed - write literal
                try self.writeFxhLiteralSymbol(@intCast(u9, chunk[0]));
                successChunk = chunk[0..1];
                // std.log.err("was lit X {}", .{chunk[0]});
            }
            self.window.inject(successChunk);
            return chunk[successChunk.len..];
        }

        // Writes a chunk. You should make chunks as big as possible, or you will waste resources.
        // In particular, this operation:
        // + Opens a new fixed-huffman block
        // + Writes the entirety of the chunk to the block
        // + Closes the Huffman block
        // The block may be marked final, which terminates the DEFLATE stream.
        // Setting final causes an automatic flush of the underlying BitWriter as no further data should be written after this point.
        // Writing data after a chunk in which the final flag has been set may cause corruption depending on the mode of operation of the framing format.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn write(self: *Self, chunk: []const u8, final: bool) BitWriterType.Error!void {
            try self.openFixedChunk(final);
            var thisChunk: []const u8 = chunk;
            while (thisChunk.len > 0) {
                thisChunk = try self.writeFixedUnit(thisChunk);
            }
            try self.closeFixedChunk(final);
        }

        // Writes a chunk as a literal block (or if required, set of literal blocks)
        // This is not an efficient method of storing data, but it ensures the underlying BitWriter can be (and will have been) flushed,
        //  without decompressor side effects such as needing to be a final block, bitstream corruption, or added output data.
        // It's important to note that passing an empty chunk is a valid way to use this if you for some reason *must* get the stream aligned without terminating it.
        // Writing data after a chunk in which the final flag has been set may cause corruption depending on the mode of operation of the framing format.
        // The final flag may be set here in order to finalize a stream which wasn't previously finalized,
        //  but this is wasteful in comparison to supplying it on the previous block.
        // Finally, the state of the DEFLATE stream is undefined if any error occurs.
        // This type will never generate an error that was not received from the underlying writers.
        pub fn writeLiteral(self: *Self, chunk: []const u8, final: bool) BitWriterType.Error!void {
            var hasWrittenOneBlock: bool = false;
            var thisChunk: []const u8 = chunk;
            while ((!hasWrittenOneBlock) or (thisChunk.len > 0)) {
                var nextChunk: []const u8 = &[0]u8{};
                if (thisChunk.len >= 0x10000) {
                    nextChunk = thisChunk[0xFFFF..];
                    thisChunk = thisChunk[0..0xFFFF];
                }
                var thisFinal = (nextChunk.len == 0) and final;
                // Final block flag.
                try self.forward_writer.writeBits(@boolToInt(thisFinal), 1);
                // Literal block - this is what gets the alignment
                try self.forward_writer.writeBits(@as(u2, 0), 2);
                // Alignment caused by literal block
                try self.forward_writer.flushBits();
                // Literal block length
                const thisChunkLen = @truncate(u16, thisChunk.len);
                try self.forward_writer.writeBits(thisChunkLen, 16);
                // Literal block length inverted
                try self.forward_writer.writeBits(~thisChunkLen, 16);
                // Flush bits just in case.
                try self.forward_writer.flushBits();
                // Time to write contents.
                try self.forward_writer.forward_writer.writeAll(thisChunk);
                // Advance
                hasWrittenOneBlock = true;
                thisChunk = nextChunk;
            }
            // Update window
            self.window.inject(chunk);
        }
    };
}


// -- Testbed --

pub fn main() anyerror!void {
    // would prefer not to allocate, but kinda have to for this bit the way it's setup
    var alloc = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const data = try std.fs.cwd().readFileAlloc(&alloc.allocator, "plscompressme.tar", 0x1000000);
    defer alloc.deinit();
    // --
    const dcType = DeflateCompressor(0x8000, std.fs.File.Writer);
    var dc = dcType.init(std.io.getStdOut().writer());
    // zlib header
    try dc.forward_writer.writeBits(@as(u16, 0x0178), 16);
    // content
    try dc.write(data, false);
    // just to test this
    try dc.writeLiteral(&[0]u8{}, true);
    // finalize
    try dc.forward_writer.writeBits(std.hash.Adler32.hash(data), 32);
}

