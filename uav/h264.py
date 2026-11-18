"""A minimal H.264 writer for the SIMULATED drone camera (never used with the real drone).

The real Tello sends an H.264 stream. To exercise exactly the same receiving and decoding code without a drone,
the simulator has to produce real H.264 too, and there is no encoder to rely on (OpenCV's builds cannot encode
H.264). This one needs no codec at all: every macroblock is stored uncompressed (the standard calls that I_PCM),
so a picture is headers followed by the raw pixel values. Any H.264 decoder reads it; it is simply large, which
does not matter on the loopback interface.

Stream layout (Annex B, as the Tello sends it): start code + SPS, start code + PPS, start code + one IDR slice per
picture. Baseline profile, 4:2:0, picture size a multiple of 16.
"""
from __future__ import annotations

import numpy as np

START = b'\x00\x00\x00\x01'


class _Bits:
    def __init__(self):
        self.bits = []

    def u(self, n, value):
        self.bits.extend((value >> (n - 1 - k)) & 1 for k in range(n))
        return self

    def ue(self, value):                       # Exp-Golomb, unsigned
        value += 1
        length = value.bit_length()
        self.bits.extend([0] * (length - 1))
        return self.u(length, value)

    def se(self, value):                       # Exp-Golomb, signed
        return self.ue(2 * value - 1 if value > 0 else -2 * value)

    def align(self, bit=0):
        while len(self.bits) % 8:
            self.bits.append(bit)
        return self

    def trailing(self):                        # rbsp_trailing_bits: a 1, then zeros to the byte boundary
        self.bits.append(1)
        return self.align()

    def bytes(self):
        assert len(self.bits) % 8 == 0
        return bytes(int(''.join(map(str, self.bits[k:k + 8])), 2) for k in range(0, len(self.bits), 8))


def _escape(payload):
    """Emulation prevention: 00 00 0x (x <= 3) inside a NAL unit becomes 00 00 03 0x."""
    out, zeros = bytearray(), 0
    for byte in payload:
        if zeros >= 2 and byte <= 3:
            out.append(3)
            zeros = 0
        out.append(byte)
        zeros = zeros + 1 if byte == 0 else 0
    return bytes(out)


class IpcmEncoder:
    """``encode(bgr_image)`` -> the bytes of one picture (SPS + PPS + IDR slice), ready to be sent."""

    def __init__(self, width, height):
        if width % 16 or height % 16:
            raise ValueError('The picture size must be a multiple of 16.')
        self.width, self.height = width, height
        self.mbs_x, self.mbs_y = width // 16, height // 16
        sps = (_Bits().u(8, 66).u(8, 0xC0).u(8, 40)         # baseline profile, constraint flags, level 4.0
               .ue(0)                                      # seq_parameter_set_id
               .ue(0)                                      # log2_max_frame_num_minus4
               .ue(2)                                      # pic_order_cnt_type 2: output order = decoding order
               .ue(1)                                      # max_num_ref_frames
               .u(1, 0)                                    # gaps_in_frame_num_value_allowed_flag
               .ue(self.mbs_x - 1).ue(self.mbs_y - 1)
               .u(1, 1)                                    # frame_mbs_only_flag
               .u(1, 1)                                    # direct_8x8_inference_flag
               .u(1, 0)                                    # frame_cropping_flag
               .u(1, 0)                                    # vui_parameters_present_flag
               .trailing().bytes())
        pps = (_Bits().ue(0).ue(0)                         # pic_parameter_set_id, seq_parameter_set_id
               .u(1, 0)                                    # entropy_coding_mode_flag: CAVLC
               .u(1, 0)                                    # bottom_field_pic_order_in_frame_present_flag
               .ue(0)                                      # num_slice_groups_minus1
               .ue(0).ue(0)                                # num_ref_idx_l0/l1_default_active_minus1
               .u(1, 0).u(2, 0)                            # weighted_pred_flag, weighted_bipred_idc
               .se(0).se(0).se(0)                          # pic_init_qp, pic_init_qs, chroma_qp_index_offset
               .u(1, 0)                                    # deblocking_filter_control_present_flag
               .u(1, 0)                                    # constrained_intra_pred_flag
               .u(1, 0)                                    # redundant_pic_cnt_present_flag
               .trailing().bytes())
        self.parameter_sets = START + b'\x67' + _escape(sps) + START + b'\x68' + _escape(pps)
        self._picture = 0

    def _slice_header(self):
        """IDR slice header followed by the first macroblock's type (I_PCM) and its alignment bits."""
        bits = (_Bits().ue(0)                              # first_mb_in_slice
                .ue(7)                                     # slice_type: I (all slices of the picture)
                .ue(0)                                     # pic_parameter_set_id
                .u(4, 0)                                   # frame_num (always 0 in an IDR picture)
                .ue(self._picture % 2)                     # idr_pic_id: differs between consecutive IDR pictures
                .u(1, 0).u(1, 0)                           # no_output_of_prior_pics_flag, long_term_reference_flag
                .se(0)                                     # slice_qp_delta
                .ue(25)                                    # mb_type 25 = I_PCM
                .align())                                  # pcm_alignment_zero_bit(s)
        return START + b'\x65' + _escape(bits.bytes())

    def encode(self, bgr):
        import cv2
        if bgr.shape[1] != self.width or bgr.shape[0] != self.height:
            raise ValueError('Picture size does not match the encoder.')
        yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
        yuv = np.maximum(yuv, 1)                           # no zero bytes in the samples: no emulation bytes needed
        h, w, my, mx = self.height, self.width, self.mbs_y, self.mbs_x
        luma = yuv[:h].reshape(my, 16, mx, 16).transpose(0, 2, 1, 3).reshape(my * mx, 256)
        cb = yuv[h:h + h // 4].reshape(h // 2, w // 2).reshape(my, 8, mx, 8).transpose(0, 2, 1, 3).reshape(my * mx, 64)
        cr = yuv[h + h // 4:].reshape(h // 2, w // 2).reshape(my, 8, mx, 8).transpose(0, 2, 1, 3).reshape(my * mx, 64)
        blocks = np.empty((my * mx, 386), np.uint8)
        blocks[:, 0], blocks[:, 1] = 0x0D, 0x00            # every further macroblock: mb_type 25 + alignment
        blocks[:, 2:258], blocks[:, 258:322], blocks[:, 322:] = luma, cb, cr
        data = self.parameter_sets + self._slice_header() + blocks.tobytes()[2:] + b'\x80'   # 0x80 = slice trailing bits
        self._picture += 1
        return data
