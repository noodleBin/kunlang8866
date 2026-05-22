import React from 'react';

import Image from 'components/common/Image';
import logoCentury from 'assets/images/logo_left_century.png';
import HMIControls from 'components/Header/HMIControls';

export default class Header extends React.Component {
  render() {
    return (
            <header className="header">
                <Image image={logoCentury} className="century-logo" />
                {!OFFLINE_PLAYBACK && <HMIControls />}
            </header>
    );
  }
}
