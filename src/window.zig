// zig-deflate-compressor - Zig zero-allocation DEFLATE (RFC 1951) Compressor
// Written starting in 2021 by contributors
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

const std = @import("std");

pub const WindowFind = struct {
    distance: u16,
    // NOTE: Maximum length is 258.
    length: u16
};

pub fn windowInjectByteDefault(self: anytype, byte: u8) void {
    var tmp = [_]u8{ byte };
    self.inject(tmp[0..1]);
}

// -- Implementations --

pub fn MemoryOptimizedWindow(comptime windowSize: u16) type {
    // Past this window size, unrepresentable distances may occur,
    //  integer overflows may occur...
    // Larger than this is also not supported decoding-wise
    std.debug.assert(windowSize <= 0x8000);
    return struct {
        // Window ring
        data: [windowSize]u8,
        // Amount of the ring which is valid
        valid: u16,
        // Write position in the ring
        ptr: u16,

        const Self = @This();
        pub fn init() Self {
            return Self {
                .data = undefined,
                .valid = 0,
                .ptr = 0,
            };
        }

        fn ptrBack(self: *Self, ptr: u16) u16 {
            _ = self;
            if (ptr == 0)
                return windowSize - 1;
            return ptr - 1;
        }

        fn ptrFwd(self: *Self, ptr: u16) u16 {
            _ = self;
            if (ptr == (windowSize - 1))
                return 0;
            return ptr + 1;
        }

        // Finds as much as it can of the given data slice within the window.
        // Note that it may return a length of 0 - this means you must use a literal.
        // Note that it will NOT return a length above the length of the input data.
        // Nor will it return a length above 258 (the maximum length).
        // Note that it may return 'extended' (off the end of the window) finds, this is normal and allowed in DEFLATE.
        pub fn find(self: *Self, dataC: []const u8) WindowFind {
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
            return WindowFind {
                .distance = bestDistance,
                .length = bestLength,
            };
        }

        // Injects a byte into the window.
        // This covers situations where some known activity was performed on the DEFLATE stream.
        pub const injectByte = windowInjectByteDefault;

        // Injects a slice into the window.
        // This covers situations where some known activity was performed on the DEFLATE stream.
        pub fn inject(self: *Self, data: []const u8) void {
            // Mostly naive method for now, with some slight adjustments
            if (data.len > windowSize) {
                self.inject(data[(data.len - windowSize)..data.len]);
                return;
            }
            for (data) |b| {
                self.data[self.ptr] = b;
                self.ptr = self.ptrFwd(self.ptr);
            }
            if ((windowSize - self.valid) < data.len) {
                self.valid = windowSize;
            } else {
                // It's important to note that the data.len vs. windowSize check above prevents this from triggering UB.
                self.valid += @intCast(u16, data.len);
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

