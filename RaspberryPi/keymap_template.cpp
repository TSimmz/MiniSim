const OneFx::BooleanParameter * mKeys[IFF_KEY_SIZE] ;
bool mPrevKeys[IFF_KEY_SIZE] ;

  int i = 0 ;

  mKeys[i++] = &mKey_LSK_L1 ;
  mKeys[i++] = &mKey_LSK_L2 ;
  mKeys[i++] = &mKey_LSK_L3 ;
  mKeys[i++] = &mKey_LSK_L4 ;
  mKeys[i++] = &mKey_LSK_L5 ;
  mKeys[i++] = &mKey_LSK_R1 ;
  mKeys[i++] = &mKey_LSK_R2 ;
  mKeys[i++] = &mKey_LSK_R3 ;
  mKeys[i++] = &mKey_LSK_R4 ;
  mKeys[i++] = &mKey_LSK_R5 ;

  mKeys[i++] = &mStatKey ;
  mKeys[i++] = &mModeKey ;
  mKeys[i++] = &mHomeKey ;
  mKeys[i++] = &mIdentKey ;
  mKeys[i++] = &mSelectPress ;

  i++;

  mSelEncoder_prev = mSelEncoder;

//  ===========================================================================
//
/// whichKeyPressed
//
//  ===========================================================================
int IFFKeyPadPanel::whichKeyPressed ( void )
{
  int wReturnKey = C12_IFF_PANEL_KEYS::eKeyUndef ;

  mPressKey   = C12_IFF_PANEL_KEYS::eKeyUndef ;
  mHoldKey    = C12_IFF_PANEL_KEYS::eKeyUndef ;
  mReleaseKey = C12_IFF_PANEL_KEYS::eKeyUndef ;
    
  // Check if LSK key is pressed
  if ( mKey_LSK_L1 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L1] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_L1 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_L2 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L2] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_L2 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_L3 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L3] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_L3 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_L4 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L4] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_L4 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_L5 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L5] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_L5 ; mLSKTimer->restartTimer(mLSKDelay);} 
  if ( mKey_LSK_R1 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R1] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_R1 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_R2 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R2] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_R2 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_R3 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R3] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_R3 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_R4 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R4] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_R4 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mKey_LSK_R5 && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R5] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_LSK_R5 ; mLSKTimer->restartTimer(mLSKDelay);}
  if ( mStatKey    && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_STAT  ] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_STAT   ; } 
  if ( mModeKey    && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_MODE  ] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_MODE   ; } 
  if ( mHomeKey    && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_HOME  ] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_HOME   ; }  
  if ( mIdentKey   && !mPrevKeys[C12_IFF_PANEL_KEYS::eKey_IDENT ] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eKey_IDENT  ; } 
  if ( mSelectPress&& !mPrevKeys[C12_IFF_PANEL_KEYS::eSelEncBut ] ) { wReturnKey = mPressKey = C12_IFF_PANEL_KEYS::eSelEncBut  ; mLSKTimer->restartTimer(mLSKDelay);}

  // Check if LSK key is held
  if ( mKey_LSK_L1 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L1] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_L1 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_L2 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L2] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_L2 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_L3 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L3] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_L3 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_L4 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L4] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_L4 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_L5 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L5] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_L5 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_R1 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R1] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_R1 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_R2 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R2] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_R2 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_R3 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R3] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_R3 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_R4 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R4] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_R4 ; mLSKTimer->isTimerExpired(); }
  if ( mKey_LSK_R5 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R5] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eKey_LSK_R5 ; mLSKTimer->isTimerExpired(); }
  if ( mSelectPress&&  mPrevKeys[C12_IFF_PANEL_KEYS::eSelEncBut ] ) { wReturnKey = mHoldKey = C12_IFF_PANEL_KEYS::eSelEncBut  ; mLSKTimer->isTimerExpired(); }
  
  // Verify held key
  if ( mHoldKey != C12_IFF_PANEL_KEYS::eKeyUndef ) { mPressKey = C12_IFF_PANEL_KEYS::eKeyUndef; }

  // Check if LSK key is released
  if (!mKey_LSK_L1 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L1] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_L1 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_L2 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L2] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_L2 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_L3 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L3] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_L3 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_L4 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L4] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_L4 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_L5 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_L5] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_L5 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_R1 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R1] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_R1 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_R2 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R2] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_R2 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_R3 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R3] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_R3 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_R4 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R4] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_R4 ; mLSKTimer->stopTimer(); }
  if (!mKey_LSK_R5 &&  mPrevKeys[C12_IFF_PANEL_KEYS::eKey_LSK_R5] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eKey_LSK_R5 ; mLSKTimer->stopTimer(); }
  if (!mSelectPress&&  mPrevKeys[C12_IFF_PANEL_KEYS::eSelEncBut ] ) { wReturnKey = mReleaseKey = C12_IFF_PANEL_KEYS::eSelEncBut  ; mLSKTimer->stopTimer(); }
  
  // Verify release key
  if ( mReleaseKey != C12_IFF_PANEL_KEYS::eKeyUndef ) { mHoldKey = C12_IFF_PANEL_KEYS::eKeyUndef; }

  // zeroize switch toggled
  //if ( mZeroToggle) 
  //{
  //  // Zero toggle switch on - zeroize
  //}

  // Check the select encoder
  if (mSelEncoder != mSelEncoder_prev)
  {
    if (mSelEncoder > mSelEncoder_prev) // Clockwise
    {
      mIFFDisplay[mCurrentPage]->setSelectKnob(CW, (mSelEncoder - mSelEncoder_prev));
    }
    else                                // Counter-clockwise
    {
      mIFFDisplay[mCurrentPage]->setSelectKnob(CCW, (mSelEncoder_prev - mSelEncoder));
    }

    // Reset page timer when knob moves
    if (mCurrentPage != eHome)
    {
      mPageTimer->restartTimer(mPageDelay);
    }

    mSelEncoder_prev = mSelEncoder;
  }
  else
  {
    mIFFDisplay[mCurrentPage]->setSelectKnob(CW, 0);
  }

  // Goes to test page
  if ( mStatKey && mModeKey )
  {
    setDisplayPage(eTest);
  }

  // save previous keys
  savePrevious();

  return wReturnKey ;
}

//  ===========================================================================
//
/// handleKeys
//
//  ===========================================================================
int IFFKeyPadPanel::handleKeys ( void )
{
  int wCurrentKey = C12_IFF_PANEL_KEYS::eKeyUndef ;
  wCurrentKey= whichKeyPressed() ;

  switch (wCurrentKey)
  {
    case C12_IFF_PANEL_KEYS::eKey_LSK_L1:
    case C12_IFF_PANEL_KEYS::eKey_LSK_L2:
    case C12_IFF_PANEL_KEYS::eKey_LSK_L3:
    case C12_IFF_PANEL_KEYS::eKey_LSK_L4:
    case C12_IFF_PANEL_KEYS::eKey_LSK_L5:
    case C12_IFF_PANEL_KEYS::eKey_LSK_R1:
    case C12_IFF_PANEL_KEYS::eKey_LSK_R2:
    case C12_IFF_PANEL_KEYS::eKey_LSK_R3:
    case C12_IFF_PANEL_KEYS::eKey_LSK_R4:
    case C12_IFF_PANEL_KEYS::eKey_LSK_R5:
    case C12_IFF_PANEL_KEYS::eSelEncBut:
      // LSK Button functions
      // 1) <  2 seconds
      // 2. >= 2 seconds
      if (mInTest == true && mPressKey == C12_IFF_PANEL_KEYS::eKey_LSK_L2)
      {
        runTestLighting(false);
      }
      else
      {

        mIFFDisplay[mCurrentPage]->HandleKeys(static_cast<C12_IFF_PANEL_KEYS::C12_IFF_PANEL_KEYS>(wCurrentKey),
                                              static_cast<C12_IFF_PANEL_KEYS::C12_IFF_PANEL_KEYS>(mPressKey),
                                              static_cast<C12_IFF_PANEL_KEYS::C12_IFF_PANEL_KEYS>(mHoldKey),
                                              static_cast<C12_IFF_PANEL_KEYS::C12_IFF_PANEL_KEYS>(mReleaseKey),
                                              mLSKTimer->expired);
      }
      break;
    case C12_IFF_PANEL_KEYS::eKey_STAT    :
      // Switches between 3 status screens
      // 1. Mode status 1/3
      // 2. Mode status 2/3
      // 3. Mode status 3/3
      // Wraps around
      if (mCurrentPage == eStatTwo)
      {
        setDisplayPage(eStatThree);
      }
      else if (mCurrentPage == eStatOne)
      {
        setDisplayPage(eStatTwo);
      }
      else
      {
        setDisplayPage(eStatOne);
      }
      break;
    case C12_IFF_PANEL_KEYS::eKey_MODE    :
      // Displays the MODE screen
      setDisplayPage(eMode);
      break;
    case C12_IFF_PANEL_KEYS::eKey_HOME    :
      // Displays the HOME screen
      // If pushed with MODE key, then maintenance menu is displayed

      // switch page to HOME page unless on home page
      setDisplayPage(eHome);
      break;
    case C12_IFF_PANEL_KEYS::eKey_IDENT    :

      mIdentTimer->restartTimer(mIdentDelay);
      
      //mIFFDisplay[mCurrentPage]->checkIdentStatus(mIdentTimer->isTimerExpired());

      break;
  }

  //Check external ident
  if (mIdentExternal == true)
  {
    mIdentTimer->restartTimer(mIdentDelay);
  }

  if (mIFFDisplay[mCurrentPage]->isPageChanged())
  {
    mCurrentPage = mIFFDisplay[mCurrentPage]->getCurrentPage();
    
    // If display not HOME, restart the timer
    if (mCurrentPage != eHome)
    {
      mPageTimer->restartTimer(mPageDelay);
    }

    // Stop timer if at HOME
    else
    {
      mPageTimer->stopTimer();
    }

    mIFFDisplay[mCurrentPage]->Initialize();
  }
  else
  {
    // If a button was pressed, reset the timer
    if ( mCurrentPage != eHome && wCurrentKey != C12_IFF_PANEL_KEYS::eKeyUndef )
    {
      mPageTimer->restartTimer(mPageDelay);
    }

    // If not HOME and no buttons pressed, go HOME
    if (mCurrentPage != eHome && mPageTimer->isTimerExpired() && wCurrentKey == C12_IFF_PANEL_KEYS::eKeyUndef)
    {
      setDisplayPage(eHome);
    }
  }

  return 0;

}