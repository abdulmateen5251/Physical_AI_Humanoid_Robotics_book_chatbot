import React from 'react';
import OriginalNavbarContent from '@theme-original/Navbar/Content';
import AuthButton from '@site/src/components/AuthButton';

export default function NavbarContent(props) {
  return (
    <div style={{ display: 'flex', alignItems: 'center', width: '100%' }}>
      <OriginalNavbarContent {...props} />
      <div style={{ marginLeft: 'auto', paddingRight: '16px' }}>
        <AuthButton />
      </div>
    </div>
  );
}
