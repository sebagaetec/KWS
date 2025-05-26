/**
  ******************************************************************************
  * @file    sine_model_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-04-18T20:54:49-0400
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "sine_model_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_sine_model_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_sine_model_weights_array_u64[161] = {
  0xbf02c4933eb2b160U, 0xbe686454bf123335U, 0x3e85e438baa4f400U, 0xbef51695bde9ac44U,
  0xbf011ec6bef6f82fU, 0xbed41be13ee55c31U, 0x3e8881a13e814797U, 0xbe0e83163f08e62aU,
  0xbc905e3aU, 0x0U, 0xbf16ecc200000000U, 0x0U,
  0x0U, 0xbf500367U, 0x3f90ea78bf1172ccU, 0xbda2d674U,
  0x3d87b9983d89659dU, 0xbbe9b200be22a1a8U, 0xbb643521beab0818U, 0x3dc82af43d3685a8U,
  0x3e6c2b223d2fc318U, 0x3e7656c23ed85b6dU, 0xbe4c19d43eccd7deU, 0xbe0c39063e9944e9U,
  0xbeb50180be1aa87fU, 0x3ecced193d909cf4U, 0xbf228285be02c50cU, 0xbe3546763e114396U,
  0x3e2786aa3d431c58U, 0xbeae49b9bca43393U, 0x3ee96ca8bde5eda4U, 0xbd596da03e4fe7ccU,
  0xbe913438be4d0ea5U, 0xbedadbcfbd5c9e48U, 0xbe0d6744bec48c55U, 0x3e9ad97d3ec02dd5U,
  0xbe9b7234be5800f0U, 0xbeca72a43de7d5b4U, 0xbe9ec0b8bec63d7bU, 0x3db182743e83f389U,
  0x3de8d9043eaa3c53U, 0x3e25cc12be111008U, 0x3d4cc6c63e9a0b55U, 0x3e93a0b9bdf3e0e8U,
  0x3e8ea2f3be7e2b01U, 0x3ec33d01be8f805fU, 0x3f201102be9bca3eU, 0x3d19c800be1254e7U,
  0x3e270b0abe43e795U, 0x3d8286043eaf55f9U, 0x3eb9058fbd31cb98U, 0xbebe747cbec0a860U,
  0x3e9caea53d8304e4U, 0xbea6836cbed045f3U, 0xbe2d177d3e4e24e2U, 0x3db646ec3ea4aa95U,
  0x3eb75989beca12ccU, 0x3e0fb4eabdf372f8U, 0xbec3edc53edbb019U, 0xbe16af4abe3abef4U,
  0x3ea200633ded0404U, 0x3e8ce413bda4923cU, 0xbdb5b01c3eb01109U, 0xbe13bdb6bc6584c0U,
  0xbe6a842e3ea7d193U, 0xbe069a80be34b33cU, 0xbe0476eabe686907U, 0x3e964bbfbe5bcc3eU,
  0x3ec3601b3ec3f4e7U, 0xba6920003e4d2cd7U, 0xbe1eb3893ed30ae8U, 0xbed537413ea22ab9U,
  0x3d8b051c3e3486d9U, 0xbd5e1480bec37afbU, 0xbf24c7b33ca8ed30U, 0xbeb3eb0fbe18e606U,
  0xbe69f1f53edcdc09U, 0x3e0b3ffebf13250eU, 0x3eeda2f7bf891e85U, 0x3ea9baf9be7db5a3U,
  0xbbf6cf80be3412d4U, 0xbe897087bc633860U, 0xbec018f1be07f504U, 0x3e1abc36be4b6bf9U,
  0x3e9ffabfbe162e8cU, 0x3e3aef2ebe9dc2ccU, 0xbec942ec3e8d6639U, 0xbe0c6160bec5b38dU,
  0xbedaaedfbe20a7bcU, 0x3e7035b2be9c07c5U, 0x3e2721f63e4c852aU, 0x3e2a3d363ebd60b9U,
  0x3e6966aa3e5119d6U, 0xbd51a3e83eb85de5U, 0xbe40810a3cb05e40U, 0x3d1207d0bece3478U,
  0x3ec47f873e769a96U, 0x3e7eca5abe2be0a4U, 0x3eccfb6e3e34152aU, 0xbeb362753ebd9bedU,
  0xbc6c17803eb4e7e5U, 0xbd4f5da03ee996d3U, 0xbecf2c1a3eb93d72U, 0x3e3d008abc9ca12cU,
  0xbeaa2e503e202b2aU, 0xbe5009f73e638edaU, 0x3ed61a20bea41644U, 0x3e194602be261e3cU,
  0x3e890485bec46957U, 0xbc8569b0bd8a2c1bU, 0xbe91b2543ee72a51U, 0x3e0f53463e71ac7dU,
  0x3e8251cd3dbfa839U, 0xbdbd97343c572040U, 0xbebe6454bd0bc578U, 0x3d86305cbd94f058U,
  0xbe4a6d18be59c755U, 0xbc3279c03ccc5469U, 0x3c2abd0f3e060755U, 0x3e51597ebe261ee8U,
  0xbd4157183eb6127bU, 0x3ea2c3cb3bc31180U, 0xbf32611f3dd6b78cU, 0xbec91e6a3d4b7600U,
  0xbdd463903ec7e8a9U, 0xbd40db20be8e6eb1U, 0x3dd3c952be0d909cU, 0x3d967784bdc59048U,
  0x3ea43227bdf303ffU, 0xbece1f02bc0d9160U, 0xbea5257cbe8c32e4U, 0xbe0c89f8bebdbd22U,
  0xbdccce28be5ae4a2U, 0xbd6d0058be66d80bU, 0x3def8834be8b6432U, 0x3e7386c2bdd5ff6dU,
  0xbe66f411bf1f6907U, 0xbe373bc03eaaa375U, 0xbcf626f8bd8f2058U, 0xbe4ef6833d283700U,
  0x3ec48749be1ee48aU, 0xbed6be39bf2e91a5U, 0xbd095d70be4dcb69U, 0xbe55cdffbf76f80dU,
  0x3eb0628ebe82bbf2U, 0x3ec9f3b300000000U, 0xbcb9ff70U, 0x3f5c9e84be98a496U,
  0x0U, 0xbec0ce65bf0c8107U, 0x3ebd8a52bcd57859U, 0x3f49c25dbd010e16U,
  0xbf0d2c913e0ae9f4U, 0xbed528f53e5a0220U, 0x3da4b5483e25f801U, 0x3fbb07703eab6ff4U,
  0x3ef239183f06ee1cU, 0x3f02a3af3f14cbb0U, 0x3f025c9fbee07910U, 0xbf9604d8be4278e8U,
  0xbea33119U,
};


ai_handle g_sine_model_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_sine_model_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

