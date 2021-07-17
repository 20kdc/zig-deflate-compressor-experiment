// zig-deflate-compressor - Zig zero-allocation DEFLATE (RFC 1951) Compressor
// Written starting in 2021 by contributors
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

const std = @import("std");

const window = @import("./window.zig");
const compressor = @import("./compressor.zig");

// -- Testbed --

pub fn main() anyerror!void {
    // confirms this compiles (ought to be tests really)
    var mt = window.MemoryOptimizedWindow(0x8000).init();
    mt.injectByte(2);
    // would prefer not to allocate, but kinda have to for this bit the way it's setup
    var alloc = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const data = try std.fs.cwd().readFileAlloc(&alloc.allocator, "plscompressme.tar", 0x1000000);
    defer alloc.deinit();
    // --
    const dcType = compressor.DeflateCompressor(window.MemoryOptimizedWindow(0x8000), std.fs.File.Writer);
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

