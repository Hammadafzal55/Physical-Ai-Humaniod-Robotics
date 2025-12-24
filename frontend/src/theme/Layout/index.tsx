import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { ChatProvider } from '../../context/ChatContext';
import ChatLauncher from '../../components/ChatLauncher'; // Import ChatLauncher
import ChatWindow from '../../components/ChatWindow';   // Import ChatWindow

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <ChatProvider>
      <Layout {...props} />
      <ChatLauncher /> {/* Render ChatLauncher */}
      <ChatWindow />   {/* Render ChatWindow */}
    </ChatProvider>
  );
}
