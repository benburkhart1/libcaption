/*
 * Frame-based TS caption injection
 * Processes complete frames as units to properly handle SEI injection
 */

#include "srt.h"
#include "ts.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t* data;
    size_t size;
    size_t capacity;
} buffer_t;

void buffer_init(buffer_t* buf) {
    memset(buf, 0, sizeof(buffer_t));
    buf->capacity = 65536; // 64KB initial
    buf->data = malloc(buf->capacity);
}

void buffer_free(buffer_t* buf) {
    free(buf->data);
    memset(buf, 0, sizeof(buffer_t));
}

void buffer_append(buffer_t* buf, const uint8_t* data, size_t size) {
    while (buf->size + size > buf->capacity) {
        buf->capacity *= 2;
        buf->data = realloc(buf->data, buf->capacity);
    }
    memcpy(buf->data + buf->size, data, size);
    buf->size += size;
}

void buffer_reset(buffer_t* buf) {
    buf->size = 0;
}

// Write ES data as TS packets
int write_es_as_ts_packets(FILE* output, const uint8_t* es_data, size_t es_size,
                           uint16_t pid, uint8_t cc_start, int write_pusi,
                           const uint8_t* pes_header, size_t pes_header_size) {
    size_t es_offset = 0;
    uint8_t cc = cc_start;
    int packets_written = 0;
    
    if (write_pusi && pes_header && pes_header_size > 0) {
        // Write PUSI packet with PES header
        uint8_t pkt[188];
        memset(pkt, 0xFF, 188);
        
        pkt[0] = 0x47;
        pkt[1] = 0x40 | ((pid >> 8) & 0x1F); // PUSI flag
        pkt[2] = pid & 0xFF;
        pkt[3] = 0x10 | (cc & 0x0F);
        
        // Copy PES header
        memcpy(pkt + 4, pes_header, pes_header_size);
        
        // Copy ES data
        size_t payload_space = 184 - pes_header_size;
        size_t to_copy = (es_size < payload_space) ? es_size : payload_space;
        memcpy(pkt + 4 + pes_header_size, es_data, to_copy);
        
        fwrite(pkt, 188, 1, output);
        es_offset = to_copy;
        cc = (cc + 1) & 0x0F;
        packets_written++;
    }
    
    // Write continuation packets
    while (es_offset < es_size) {
        uint8_t pkt[188];
        memset(pkt, 0xFF, 188);
        
        pkt[0] = 0x47;
        pkt[1] = (pid >> 8) & 0x1F;
        pkt[2] = pid & 0xFF;
        
        size_t remaining = es_size - es_offset;
        if (remaining >= 184) {
            // Full payload
            pkt[3] = 0x10 | (cc & 0x0F);
            memcpy(pkt + 4, es_data + es_offset, 184);
            es_offset += 184;
        } else {
            // Last packet with padding - use adaptation field
            int adapt_len = 183 - remaining;
            pkt[3] = 0x30 | (cc & 0x0F); // Has adaptation and payload
            pkt[4] = adapt_len;
            if (adapt_len > 1) {
                pkt[5] = 0x00; // No flags
                // Rest is already 0xFF padding
            }
            memcpy(pkt + 5 + adapt_len, es_data + es_offset, remaining);
            es_offset += remaining;
        }
        
        fwrite(pkt, 188, 1, output);
        cc = (cc + 1) & 0x0F;
        packets_written++;
    }
    
    return packets_written;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s <input.ts> <captions.srt> <output.ts>\n", argv[0]);
        return EXIT_FAILURE;
    }

    // Load SRT
    FILE* srt_file = fopen(argv[2], "rb");
    if (!srt_file) {
        fprintf(stderr, "Failed to open SRT file: %s\n", argv[2]);
        return EXIT_FAILURE;
    }
    
    fseek(srt_file, 0, SEEK_END);
    size_t srt_size = ftell(srt_file);
    fseek(srt_file, 0, SEEK_SET);
    
    char* srt_data = malloc(srt_size + 1);
    fread(srt_data, 1, srt_size, srt_file);
    srt_data[srt_size] = '\0';
    fclose(srt_file);
    
    srt_t* srt = srt_parse(srt_data, srt_size);
    free(srt_data);
    
    if (!srt || !srt->cue_head) {
        fprintf(stderr, "No captions in SRT\n");
        return EXIT_FAILURE;
    }
    
    FILE* input = fopen(argv[1], "rb");
    FILE* output = fopen(argv[3], "wb");
    
    if (!input || !output) {
        fprintf(stderr, "Failed to open files\n");
        return EXIT_FAILURE;
    }
    
    fprintf(stderr, "Processing %s -> %s with captions from %s\n", 
            argv[1], argv[3], argv[2]);
    
    ts_t ts;
    ts_init(&ts);
    
    uint8_t pkt[188];
    srt_cue_t* current_cue = srt->cue_head;
    int16_t video_pid = 0;
    double offset = 0.0;
    int captions_injected = 0;
    
    // Frame tracking
    buffer_t frame_packets;
    buffer_t frame_es;
    buffer_init(&frame_packets);
    buffer_init(&frame_es);
    
    uint8_t pes_header[256];
    size_t pes_header_size = 0;
    int in_video_frame = 0;
    uint8_t frame_cc_start = 0;
    double frame_timestamp = 0.0;
    int frame_number = 0;  // Track which frame we're on
    
    // Caption splitting state
    caption_frame_t current_caption_frame;
    caption_frame_init(&current_caption_frame);
    int caption_frames_remaining = 0;
    int current_caption_chunk = 0;
    
    while (fread(pkt, 188, 1, input) == 1) {
        int16_t pid = ((pkt[1] & 0x1F) << 8) | pkt[2];
        int is_pusi = (pkt[1] & 0x40) ? 1 : 0;
        
        // Parse packet to track video stream
        int parsed = ts_parse_packet(&ts, pkt);
        
        // Identify video PID
        if (!video_pid && parsed == LIBCAPTION_READY && ts.data && ts.size > 0) {
            video_pid = pid;
            fprintf(stderr, "Video PID: 0x%04x\n", video_pid);
        }
        
        // Handle video packets
        if (video_pid && pid == video_pid) {
            if (is_pusi) {
                // Process previous frame if complete
                if (in_video_frame && frame_packets.size > 0) {
                    frame_number++;
                    // Determine if we should inject caption
                    int should_inject = 0;
                    
                    // Check if we need to start a new caption
                    if (current_cue && offset > 0.0 && caption_frames_remaining == 0) {
                        double caption_time = current_cue->timestamp + offset;
                        if (caption_time <= frame_timestamp) {
                            // Initialize caption for splitting
                            caption_frame_from_text(&current_caption_frame, srt_cue_data(current_cue));
                            caption_frames_remaining = 2; // Split across 2 frames like working file
                            current_caption_chunk = 0;
                            fprintf(stderr, "Starting caption split across %d frames: %s\n", 
                                    caption_frames_remaining, srt_cue_data(current_cue));
                        }
                    }
                    
                    // Check if we should inject part of current caption
                    if (caption_frames_remaining > 0 && frame_number <= 2) {
                        should_inject = 1;
                    }
                    
                    if (should_inject && frame_es.size > 0) {
                        // Create partial caption SEI based on chunk
                        caption_frame_t chunk_frame;
                        caption_frame_init(&chunk_frame);
                        
                        // Split text like working file pattern
                        const char* full_text = srt_cue_data(current_cue);
                        char partial_text[256];
                        
                        // Split text like output_new pattern (9 CC pairs + 6 CC pairs)
                        if (current_caption_chunk == 0) {
                            // First chunk: take first half of text (like output_new frame 0)
                            size_t text_len = strlen(full_text);
                            size_t first_half = text_len / 2;
                            strncpy(partial_text, full_text, first_half);
                            partial_text[first_half] = '\0';
                            fprintf(stderr, "DEBUG: Creating SEI chunk 0 (first half) with PTS %.3f: '%s'\n", 
                                    frame_timestamp, partial_text);
                        } else {
                            // Second chunk: take second half of text (like output_new frame 1)
                            size_t text_len = strlen(full_text);
                            size_t first_half = text_len / 2;
                            strcpy(partial_text, full_text + first_half);
                            fprintf(stderr, "DEBUG: Creating SEI chunk 1 (second half) with PTS %.3f: '%s'\n", 
                                    frame_timestamp, partial_text);
                        }
                        
                        caption_frame_from_text(&chunk_frame, partial_text);
                        
                        sei_t sei;
                        // Use the same timestamp for all chunks so they're treated as one caption
                        static double caption_start_timestamp = 0.0;
                        if (current_caption_chunk == 0) {
                            caption_start_timestamp = frame_timestamp;
                        }
                        sei_init(&sei, caption_start_timestamp);
                        sei_from_caption_frame(&sei, &chunk_frame);
                        current_caption_chunk++;
                        caption_frames_remaining--;
                        
                        size_t sei_alloc_size = sei_render_size(&sei);
                        uint8_t* sei_data = malloc(sei_alloc_size);
                        size_t sei_size = sei_render(&sei, sei_data);
                        fprintf(stderr, "DEBUG: SEI allocated %zu, actual %zu bytes\n", 
                                sei_alloc_size, sei_size);
                        fprintf(stderr, "DEBUG: Last 5 bytes of SEI: %02x %02x %02x %02x %02x\n",
                                sei_data[sei_size-5], sei_data[sei_size-4], 
                                sei_data[sei_size-3], sei_data[sei_size-2], 
                                sei_data[sei_size-1]);
                        
                        // Create SEI NALU with 3-byte start code (like working files)
                        buffer_t sei_nalu;
                        buffer_init(&sei_nalu);
                        uint8_t sei_header[] = {0x00, 0x00, 0x01};
                        buffer_append(&sei_nalu, sei_header, 3);
                        buffer_append(&sei_nalu, sei_data, sei_size);  // Use actual size, not allocated
                        // sei_render already includes the stop bit
                        
                        // Find slice in ES and inject SEI before it
                        // Look for both 3-byte and 4-byte start codes
                        size_t injection_point = 0;
                        int found_slice = 0;
                        for (size_t i = 0; i <= frame_es.size - 4; i++) {
                            uint8_t nalu_type = 0;
                            size_t nalu_start = 0;
                            
                            // Check for 4-byte start code
                            if (i <= frame_es.size - 5 &&
                                frame_es.data[i] == 0x00 && frame_es.data[i+1] == 0x00 &&
                                frame_es.data[i+2] == 0x00 && frame_es.data[i+3] == 0x01) {
                                nalu_type = frame_es.data[i+4] & 0x1F;
                                nalu_start = i;
                            }
                            // Check for 3-byte start code
                            else if (frame_es.data[i] == 0x00 && frame_es.data[i+1] == 0x00 &&
                                     frame_es.data[i+2] == 0x01) {
                                nalu_type = frame_es.data[i+3] & 0x1F;
                                nalu_start = i;
                            }
                            
                            if (nalu_type >= 1 && nalu_type <= 5) {
                                injection_point = nalu_start;
                                found_slice = 1;
                                int start_code_len = (frame_es.data[i+2] == 0x00) ? 4 : 3;
                                fprintf(stderr, "DEBUG: Found slice (type %d) with %d-byte start code at offset %zu in ES\n", 
                                        nalu_type, start_code_len, nalu_start);
                                fprintf(stderr, "DEBUG: Next 16 bytes: ");
                                for (int j = 0; j < 16 && nalu_start+j < frame_es.size; j++) {
                                    fprintf(stderr, "%02x ", frame_es.data[nalu_start+j]);
                                }
                                fprintf(stderr, "\n");
                                break;
                            }
                        }
                        
                        if (!found_slice) {
                            fprintf(stderr, "WARNING: No slice found in ES data!\n");
                        }
                        
                        // Create modified ES
                        buffer_t modified_es;
                        buffer_init(&modified_es);
                        buffer_append(&modified_es, frame_es.data, injection_point);
                        buffer_append(&modified_es, sei_nalu.data, sei_nalu.size);
                        size_t remaining = frame_es.size - injection_point;
                        fprintf(stderr, "DEBUG: Appending %zu bytes from injection point %zu\n", 
                                remaining, injection_point);
                        fprintf(stderr, "DEBUG: First 16 bytes being appended: ");
                        for (int j = 0; j < 16 && j < remaining; j++) {
                            fprintf(stderr, "%02x ", frame_es.data[injection_point + j]);
                        }
                        fprintf(stderr, "\n");
                        
                        buffer_append(&modified_es, frame_es.data + injection_point, remaining);
                        
                        fprintf(stderr, "DEBUG: Modified ES around injection (SEI then slice):\n");
                        size_t start = injection_point;
                        size_t end = injection_point + sei_nalu.size + 32;
                        if (end > modified_es.size) end = modified_es.size;
                        
                        for (size_t j = start; j < end; j++) {
                            if ((j - start) % 16 == 0) fprintf(stderr, "\n  %04zx: ", j);
                            fprintf(stderr, "%02x ", modified_es.data[j]);
                        }
                        fprintf(stderr, "\n");
                        
                        // Write modified frame as TS packets
                        fprintf(stderr, "DEBUG: Original ES size: %zu, Modified ES size: %zu\n",
                                frame_es.size, modified_es.size);
                        fprintf(stderr, "DEBUG: Original packets: %zu, Need packets: %zu\n",
                                frame_packets.size / 188, 
                                (pes_header_size + modified_es.size + 183) / 184);
                        
                        // Verify modified ES has both SEI and slice
                        fprintf(stderr, "DEBUG: Verifying modified ES contents:\n");
                        int found_sei = 0, found_slice_after = 0;
                        for (size_t i = 0; i <= modified_es.size - 4; i++) {
                            uint8_t nalu_type = 0;
                            size_t nalu_pos = 0;
                            
                            // Check for 4-byte start code
                            if (i <= modified_es.size - 5 &&
                                modified_es.data[i] == 0x00 && modified_es.data[i+1] == 0x00 &&
                                modified_es.data[i+2] == 0x00 && modified_es.data[i+3] == 0x01) {
                                nalu_type = modified_es.data[i+4] & 0x1F;
                                nalu_pos = i;
                            }
                            // Check for 3-byte start code
                            else if (modified_es.data[i] == 0x00 && modified_es.data[i+1] == 0x00 &&
                                     modified_es.data[i+2] == 0x01) {
                                nalu_type = modified_es.data[i+3] & 0x1F;
                                nalu_pos = i;
                            }
                            
                            if (nalu_type == 6) {
                                fprintf(stderr, "  SEI at position %zu\n", nalu_pos);
                                found_sei = 1;
                            } else if (nalu_type >= 1 && nalu_type <= 5 && found_sei) {
                                fprintf(stderr, "  Slice (type %d) at position %zu (after SEI)\n", nalu_type, nalu_pos);
                                found_slice_after = 1;
                                break;
                            }
                        }
                        if (!found_slice_after) {
                            fprintf(stderr, "ERROR: Slice not found after SEI in modified ES!\n");
                        }
                        
                        int packets_written = write_es_as_ts_packets(output, modified_es.data, modified_es.size,
                                             video_pid, frame_cc_start, 1, 
                                             pes_header, pes_header_size);
                        fprintf(stderr, "DEBUG: Actually wrote %d packets\n", packets_written);
                        
                        buffer_free(&modified_es);
                        buffer_free(&sei_nalu);
                        free(sei_data);
                        sei_free(&sei);
                        
                        fprintf(stderr, "Injected caption chunk %d at %.3f\n", 
                                current_caption_chunk - 1, frame_timestamp);
                        
                        // Move to next cue only when all chunks are done
                        if (caption_frames_remaining == 0) {
                            fprintf(stderr, "Completed caption: %s\n", srt_cue_data(current_cue));
                            captions_injected++;
                            current_cue = current_cue->next;
                        }
                    } else {
                        // Write original frame packets
                        fwrite(frame_packets.data, frame_packets.size, 1, output);
                    }
                }
                
                // Start new frame
                buffer_reset(&frame_packets);
                buffer_reset(&frame_es);
                in_video_frame = 1;
                // For CC, we should use the last CC from previous frame + 1
                // But since we're rebuilding packets, let's keep track properly
                frame_cc_start = pkt[3] & 0x0F;
                frame_timestamp = ts_pts_seconds(&ts);
                
                // Set offset on first frame
                if (offset == 0.0 && frame_timestamp > 0.0) {
                    offset = frame_timestamp;
                    fprintf(stderr, "Timestamp offset: %.3f\n", offset);
                }
                
                // Store packet
                buffer_append(&frame_packets, pkt, 188);
                
                // Extract PES header and ES data
                int i = 4;
                if (pkt[3] & 0x20) { // Adaptation field
                    uint8_t adapt_len = pkt[4];
                    i = 5 + adapt_len;
                }
                
                // Find and save PES header
                if (i < 188 - 9 && pkt[i] == 0x00 && pkt[i+1] == 0x00 && pkt[i+2] == 0x01) {
                    // PES start code found
                    uint8_t header_data_len = pkt[i+8];
                    pes_header_size = 9 + header_data_len;
                    memcpy(pes_header, pkt + i, pes_header_size);
                    i += pes_header_size;
                    
                    // Extract ES data
                    if (i < 188) {
                        buffer_append(&frame_es, pkt + i, 188 - i);
                    }
                }
            } else if (in_video_frame) {
                // Continuation packet
                buffer_append(&frame_packets, pkt, 188);
                
                // Extract ES data
                int i = 4;
                if (pkt[3] & 0x20) { // Adaptation field
                    uint8_t adapt_len = pkt[4];
                    if (adapt_len < 183) {
                        i = 5 + adapt_len;
                        if (pkt[3] & 0x10) { // Has payload
                            buffer_append(&frame_es, pkt + i, 188 - i);
                        }
                    }
                } else if (pkt[3] & 0x10) { // Has payload
                    buffer_append(&frame_es, pkt + 4, 184);
                }
            } else {
                // Not in frame, pass through
                fwrite(pkt, 188, 1, output);
            }
        } else {
            // Non-video packet, pass through
            fwrite(pkt, 188, 1, output);
        }
    }
    
    // Write any remaining frame
    if (in_video_frame && frame_packets.size > 0) {
        fwrite(frame_packets.data, frame_packets.size, 1, output);
    }
    
    fprintf(stderr, "Processed file, injected %d captions\n", captions_injected);
    
    buffer_free(&frame_packets);
    buffer_free(&frame_es);
    fclose(input);
    fclose(output);
    srt_free(srt);
    
    return EXIT_SUCCESS;
}