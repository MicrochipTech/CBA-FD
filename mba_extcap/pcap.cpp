#include <cstring>
#include "pcap.hpp"
#include "pipe.hpp"

#define PCAP_MAGIC 0xa1b2c3d4

uint32_t pcap_write_file_header(sPipe& pipe, int linktype, int snaplen)
{
    pcap_hdr_t file_hdr;

    file_hdr.magic = PCAP_MAGIC;
    file_hdr.version = 2 << 16u | 4u;
    file_hdr.timeoffset = 0;
    file_hdr.accuracy = 0;
    file_hdr.maxlen = snaplen;
    file_hdr.network = linktype;

    return write_pipe(pipe, reinterpret_cast<const uint8_t*>(&file_hdr), sizeof(file_hdr));
}

uint32_t pcap_write_packet(sPipe& pipe, time_t sec, uint32_t usec, uint32_t caplen, uint32_t len, const uint8_t* pd)
{
    pcaprec_hdr rec_hdr;

    rec_hdr.ts_sec = static_cast<uint32_t>(sec);
    rec_hdr.ts_usec = usec;
    rec_hdr.incl_len = caplen;
    rec_hdr.orig_len = len;

    if (!write_pipe(pipe, reinterpret_cast<const uint8_t*>(&rec_hdr), sizeof(rec_hdr)))
        return 0;

    return write_pipe(pipe, pd, caplen);
}


uint32_t pcapng_write_block(sPipe& pipe, uint32_t blockType, uint32_t blockLen, const void* data)
{
    uint8_t pad = (4u - (blockLen & 3u)) & 3u;
    
    uint8_t* buffer = new uint8_t[12llu + blockLen + pad];

    uint32_t* block_type = reinterpret_cast<uint32_t*>(&buffer[0]);
    uint32_t* block_len1 = reinterpret_cast<uint32_t*>(&buffer[4]);
    uint32_t* block_len2 = reinterpret_cast<uint32_t*>(&buffer[8llu + blockLen + pad]);

    *block_type = blockType;
    *block_len1 = blockLen;
    *block_len2 = blockLen;

    memcpy(&buffer[8], data, blockLen);         // Todo: Check if it can replaced by 2 calls to write_pipe instead of doing memcpy first
    memset(&buffer[8 + blockLen], 0, pad);

    uint32_t ret = write_pipe(pipe, buffer, 12llu + blockLen + pad);

    delete[]buffer;

    return ret;
}

uint32_t pcapng_write_section_header(sPipe& pipe)
{
    uint32_t header[7];

    header[0] = 0x0A0D0D0Au;    // PCAPNG MAGIC
    header[1] = sizeof(header); // SIZE OF HEADER
    header[2] = 0x1A2B3C4Du;    // ENDIAN MAGIC
    header[3] = 1u;             // VERSION
    header[4] = 0;              // NO OPTIONS
    header[5] = 0;              // NO OPTIONS
    header[6] = sizeof(header); // SIZE OF HEADER

    return write_pipe(pipe, header, sizeof(header));
}

static uint32_t pcapng_write_interface_desc(sPipe& pipe, uint16_t linkType, uint32_t snapLen, const void* options, uint32_t optionsLen)
{
    uint32_t header[4];

    header[0] = 1u;
    header[1] = sizeof(header) + 4u + optionsLen;
    header[2] = linkType;
    header[3] = snapLen;

    write_pipe(pipe, &header[0], sizeof(header));
    write_pipe(pipe, options, optionsLen);
    write_pipe(pipe, &header[1], sizeof(uint32_t));

    return 1;
}

uint32_t pcapng_write_interface_desc_options(sPipe& pipe, uint16_t linkType, uint32_t snapLen, const char* if_name, const char* if_desc)
{
    char padding[4] = { 0,0,0,0 };
    tlv_t head;
    std::string buffer;

    uint16_t if_name_len = static_cast<uint16_t>(strlen(if_name));
    uint16_t if_desc_len = static_cast<uint16_t>(strlen(if_desc));
    uint8_t if_name_pad = ((4u - (if_name_len & 3u)) & 3u);
    uint8_t if_desc_pad = ((4u - (if_desc_len & 3u)) & 3u);

    buffer.reserve((2u * sizeof(head)) + if_name_len + if_name_pad + if_desc_len + if_desc_pad + 1u);
    
    head.type = 2;
    head.len = if_name_len + if_name_pad;
    buffer.append((const char*)&head, sizeof(head));
    buffer.append(if_name);
    buffer.append(padding, if_name_pad);

    head.type = 3;
    head.len = if_desc_len + if_desc_pad;
    buffer.append((const char*)&head, sizeof(head));
    buffer.append(if_desc);
    buffer.append(padding, if_desc_pad);

    pcapng_write_interface_desc(pipe, linkType, snapLen, buffer.data(), static_cast<uint32_t>(buffer.size()));

    return 1;
}

uint32_t pcapng_write_enhanced_packet(sPipe& pipe, uint32_t interface_id, uint32_t tsh, uint32_t tsl, uint32_t len, const uint8_t* data)
{
    uint32_t header[7];
    uint8_t padding[4] = { 0,0,0,0 };
    uint32_t paddedLen = (4u - (len & 3u)) & 3u;

    header[0] = 6u;
    header[1] = sizeof(header) + 4u + len + paddedLen;
    header[2] = interface_id;
    header[3] = tsh;
    header[4] = tsl;
    header[5] = len;
    header[6] = len;

    uint32_t transferedBytes = 0;
    uint32_t totalDataLen = sizeof(header) + len + paddedLen + sizeof(uint32_t);

    transferedBytes += write_pipe(pipe, &header[0], sizeof(header));
    transferedBytes += write_pipe(pipe, data, len);
    transferedBytes += write_pipe(pipe, &padding[0], paddedLen);
    transferedBytes += write_pipe(pipe, &header[1], sizeof(uint32_t));

    return (transferedBytes == totalDataLen) ? 1 : 0;
}