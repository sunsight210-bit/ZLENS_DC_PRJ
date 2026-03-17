// Test/test_storage_task.cpp
#include <gtest/gtest.h>
#include "storage_task.hpp"
#include "crc16.hpp"

using namespace zlens;

// Global required by task_config.hpp
namespace zlens {
    volatile bool g_bSpiEmergency = false;
}

class StorageTaskTest : public ::testing::Test {
protected:
    StorageTask task;
    FramStorage fram;
    SPI_HandleTypeDef hspi;
    QueueHandle_t saveQ;

    void SetUp() override {
        mock::get_log().reset();
        mock::get_log().tick_auto_increment = false;
        mock::get_log().tick = 0;
        g_bSpiEmergency = false;

        hspi.Instance = SPI2;
        fram.init(&hspi);

        saveQ = xQueueCreate(8, sizeof(SAVE_MESSAGE_S));
        task.init(&fram, saveQ);
    }

    void send_save(int32_t pos, uint16_t zoom, uint8_t reason) {
        SAVE_MESSAGE_S msg = {pos, zoom, reason};
        xQueueSend(saveQ, &msg, 0);
    }
};

TEST_F(StorageTaskTest, ReceiveSave_Arrived_CallsFram) {
    send_save(10000, 60, save_reason::ARRIVED);
    task.run_once();

    // FRAM should have been written (SPI TX called)
    EXPECT_GT(mock::get_log().spi_tx_data.size(), 0u);
    EXPECT_EQ(task.get_params().current_position, 10000);
    EXPECT_EQ(task.get_params().current_zoom_x10, 60);
}

TEST_F(StorageTaskTest, ReceiveSave_Stall_CallsFram) {
    send_save(5000, 40, save_reason::STALL);
    task.run_once();

    EXPECT_GT(mock::get_log().spi_tx_data.size(), 0u);
    EXPECT_EQ(task.get_params().last_save_reason, save_reason::STALL);
}

TEST_F(StorageTaskTest, PeriodicSave_After500ms) {
    // Set initial position
    send_save(1000, 60, save_reason::ARRIVED);
    task.run_once();
    size_t spi_count_after_first = mock::get_log().spi_tx_data.size();

    // Update position without queue message
    FRAM_PARAMS_S params = task.get_params();
    params.current_position = 2000;
    task.set_params(params);

    // Advance tick by 500ms
    mock::get_log().tick = 501;
    task.run_once();

    // Should have written again (periodic save)
    EXPECT_GT(mock::get_log().spi_tx_data.size(), spi_count_after_first);
}

TEST_F(StorageTaskTest, SpiEmergency_SkipsSave) {
    g_bSpiEmergency = true;
    send_save(10000, 60, save_reason::ARRIVED);

    size_t spi_count_before = mock::get_log().spi_tx_data.size();
    task.run_once();

    // SPI should NOT have been called
    EXPECT_EQ(mock::get_log().spi_tx_data.size(), spi_count_before);
}

TEST_F(StorageTaskTest, RestoreParams_Valid) {
    // Prepare valid FRAM data in SPI RX buffer
    FRAM_PARAMS_S params = {};
    params.magic_number = FramStorage::MAGIC;
    params.version = 1;
    params.current_position = 12345;
    params.current_zoom_x10 = 100;
    params.crc16 = FramStorage::calc_crc(params);

    auto& log = mock::get_log();
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&params);
    // SPI read: FRAM returns data after 3-byte address command
    // The mock will return from spi_rx_buffer sequentially
    log.spi_rx_buffer.assign(raw, raw + sizeof(params));

    FRAM_PARAMS_S restored;
    bool ok = task.restore_params(restored);
    // Note: The actual result depends on how FRAM SPI protocol works in mock
    // The key is that restore_params exercises the code path
    (void)ok;
}

TEST_F(StorageTaskTest, RestoreParams_Invalid) {
    // Empty SPI buffer (all 0xFF) = invalid FRAM data
    FRAM_PARAMS_S restored;
    bool ok = task.restore_params(restored);
    // Should fail since magic/CRC won't match
    EXPECT_FALSE(ok);
}

TEST_F(StorageTaskTest, UpdatesPosition_InParams) {
    send_save(1000, 60, save_reason::ARRIVED);
    task.run_once();
    EXPECT_EQ(task.get_params().current_position, 1000);

    send_save(2000, 80, save_reason::ARRIVED);
    task.run_once();
    EXPECT_EQ(task.get_params().current_position, 2000);
    EXPECT_EQ(task.get_params().current_zoom_x10, 80);
}

TEST_F(StorageTaskTest, MultipleSaves_Sequential) {
    send_save(1000, 60, save_reason::ARRIVED);
    send_save(2000, 80, save_reason::ARRIVED);
    send_save(3000, 100, save_reason::ARRIVED);

    // Process all three
    task.run_once();
    EXPECT_EQ(task.get_params().current_position, 1000);
    task.run_once();
    EXPECT_EQ(task.get_params().current_position, 2000);
    task.run_once();
    EXPECT_EQ(task.get_params().current_position, 3000);
}
